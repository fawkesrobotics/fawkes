;---------------------------------------------------------------------------
;  pddl-init.clp - Initialize PDDL CLIPS interface
;
;  Created: Thu 09 Nov 2017 16:33:45 CET
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

(defglobal
	?*PDDL-INIT-CONTROL-RETRY-INTERVAL-SEC* = 1
)

(deftemplate pddl-init-control
  (slot interface-id (type STRING))
  (slot has-writer (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (multislot last-try (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
)

(defrule pddl-init-open-interfaces
  "Open blackboard interfaces to interact with the PDDL planner."
	(executive-init)
	(ff-feature-loaded blackboard)
  =>
  (blackboard-open-reading "PddlGenInterface" "pddl-gen")
  (assert (pddl-init-control (interface-id "pddl-gen")))
  (blackboard-open-reading "PddlPlannerInterface" "pddl-planner")
  (assert (pddl-init-control (interface-id "pddl-planner")))
)

(defrule pddl-init-check-if-writer-available
  "Finish initialization if a writer for the interface exists."
  ?pi <- (pddl-init-control (interface-id ?iface) (has-writer FALSE))
  ?bi <- (blackboard-interface-info (id ?iface) (has-writer TRUE))
  =>
  (retract ?bi)
  (modify ?pi (has-writer TRUE))
)

(defrule pddl-init-update-blackboard-info
  "Get blackboard info until we have all required writers."
  (time $?now)
  ?pi <- (pddl-init-control (interface-id ?iface) (has-writer FALSE)
          (last-try $?lt&:
            (timeout ?now ?lt ?*PDDL-INIT-CONTROL-RETRY-INTERVAL-SEC*)))
  =>
  (modify ?pi (last-try ?now))
  (blackboard-get-info)
)

(defrule pddl-init-register-trigger
  "Register for the robot memory trigger for new plans."
  (executive-init)
  (ff-feature-loaded robot_memory)
  (not (registered-trigger "robmem.pddl-plan" ?))
  =>
  (printout t "Registering robot memory trigger for new plans" crlf)
  (bind ?query (bson-create))
  (bson-append ?query "plan" 1)
  (bind ?trigger (robmem-trigger-register "robmem.pddl-plan" ?query "new-plan"))
  (bson-destroy ?query)
  (assert (registered-trigger "robmem.pddl-plan" ?trigger))
)

(defrule pddl-init-pddl-interface
  ?pg <- (pddl-init-control (interface-id "pddl-gen") (has-writer TRUE))
  ?pp <- (pddl-init-control (interface-id "pddl-planner") (has-writer TRUE))
  (registered-trigger "robmem.pddl-plan" ?)
  =>
  (retract ?pg)
  (retract ?pp)
  (path-load "pddl.clp")
  (assert (ff-feature-loaded pddl_planner))
)
