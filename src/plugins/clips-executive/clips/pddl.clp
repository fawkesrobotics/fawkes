;---------------------------------------------------------------------------
;  pddl.clp - Interface to a PDDL planner using robot memory
;
;  Created: Tue 07 Nov 2017 18:36:08 CET
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

(deftemplate pddl-plan
  (slot gen-id (type INTEGER))
  (slot plan-id (type INTEGER))
  (slot status (type SYMBOL)
    (allowed-values GEN-PENDING GEN-RUNNING GENERATED PENDING RUNNING PLANNED)
  )
  (slot goal (type STRING))
)

(deffunction pddl-call (?goal)
  "Call the PDDL planner with the goal given as string."
  (bind ?m
    (blackboard-create-msg "PddlGenInterface::pddl-gen" "GenerateMessage")
  )
  (blackboard-set-msg-field ?m "goal" ?goal)
  (printout info "Calling PDDL planner for goal " ?goal crlf)
  (bind ?gen-id (blackboard-send-msg ?m))
  (assert (pddl-plan (goal ?goal) (status GEN-PENDING) (gen-id ?gen-id)))
)

(defrule pddl-check-if-generation-running
  "Check whether the PDDL generator started."
  ?p <- (pddl-plan (status GEN-PENDING) (gen-id ?gen-id))
  (PddlGenInterface (id "pddl-gen") (msg_id ?gen-id))
  =>
  (printout t "PDDL problem generation started for ID " ?gen-id crlf)
  (modify ?p (status GEN-RUNNING))
)

(defrule pddl-check-if-generation-finished
  "Check whether the PDDL generator finished."
  ?p <- (pddl-plan (status GEN-RUNNING) (gen-id ?gen-id))
  (PddlGenInterface (id "pddl-gen") (msg_id ?gen-id) (final TRUE))
  =>
  (printout t "PDDL problem generation finished for ID " ?gen-id crlf)
  (modify ?p (status GENERATED))
)

(defrule pddl-start-planner
  "Start the actual planner after generating the PDDL problem."
  ?p <- (pddl-plan (status GENERATED) (gen-id ?gen-id))
  =>
  (printout t "Starting to plan " ?gen-id crlf)
  (bind ?m
    (blackboard-create-msg "PddlPlannerInterface::pddl-planner" "PlanMessage")
  )
  (bind ?plan-id (blackboard-send-msg ?m))
  (modify ?p (plan-id ?plan-id) (status PENDING))
)

(defrule pddl-check-if-planner-running
  "Check whether the planner started to plan."
  ?p <- (pddl-plan (status PENDING) (plan-id ?plan-id))
  (PddlPlannerInterface (id "pddl-planner") (msg_id ?plan-id))
  =>
  (modify ?p (status RUNNING))
)

(defrule pddl-check-if-planner-finished
  "Check whether the planner finished planning."
  ?p <- (pddl-plan (status RUNNING) (plan-id ?plan-id))
  (PddlPlannerInterface (id "pddl-planner") (msg_id ?plan-id) (final TRUE))
  =>
  (modify ?p (status PLANNED))
)
