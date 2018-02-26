
;---------------------------------------------------------------------------
;  init.clp - CLIPS executive
;
;  Created: Tue Sep 19 16:49:42 2017
;  Copyright  2012-2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ?*CONFIG_PREFIX* = "/clips-executive"
)

(defrule executive-load-config
  (executive-init)
  =>
  (config-load ?*CONFIG_PREFIX*)
)

; (defrule executive-load-executive
;   (executive-init)
;   (confval (path "/clips-executive/spec") (type STRING) (value ?v))
;   =>
;   (printout t "Loading executive spec '" ?v "'" crlf)
;   (bind ?executive-file (path-resolve (str-cat ?v ".clp")))
;   (if ?executive-file
;     then (batch* ?executive-file)
;     else (printout logerror "Cannot find executive spec " ?v crlf))
; )

(defrule executive-enable-debug
  (executive-init)
  (confval (path "/clips-executive/clips-debug") (type BOOL) (value TRUE))
  =>
  (printout t "CLIPS debugging enabled, watching facts and rules" crlf)
  (watch facts)
  (watch rules)
  ;(dribble-on "trace.txt")
)

(defrule executive-debug-level
  (executive-init)
  (confval (path "/clips-executive/debug-level") (type UINT) (value ?v))
  =>
  (printout t "Setting debug level to " ?v " (was " ?*DEBUG* ")" crlf)
  (debug-set-level ?v)
)

(defrule executive-silence-debug-facts
  (declare (salience -1000))
  (executive-init)
  (confval (path "/clips-executive/clips-debug") (type BOOL) (value TRUE))
  (confval (path "/clips-executive/unwatch-facts") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following facts: " ?lv crlf)
  (bind ?deftemplates (get-deftemplate-list))
  (foreach ?v ?lv
    (bind ?v-sym (sym-cat ?v))
    (if (member$ ?v-sym ?deftemplates)
     then (unwatch facts ?v-sym)
     else (printout warn "Cannot unwatch " ?v " (deftemplate not defined)" crlf)
    )
  )
)

(defrule executive-silence-debug-rules
  (declare (salience -1000))
  (executive-init)
  (confval (path "/clips-executive/clips-debug") (type BOOL) (value TRUE))
  (confval (path "/clips-executive/unwatch-rules") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following rules: " ?lv crlf)
  (bind ?defrules (get-defrule-list))
  (foreach ?v ?lv
    (bind ?v-sym (sym-cat ?v))
    (if (member$ ?v-sym ?defrules)
     then (unwatch rules ?v-sym)
     else (printout warn "Cannot unwatch " ?v " (defrule not defined)" crlf)
    )
  )
)

(defrule executive-init-stage1
	(executive-init)
	=>
  (printout t "PDDL feature" crlf)
  (ff-feature-request "pddl-parser")
  (printout t "Blackboard feature and skill exec init" crlf)
	(ff-feature-request "blackboard")
)

(defrule executive-conditional-navgraph-init
  (executive-init)
  (confval (path "/clips-executive/use_navgraph") (type BOOL) (value TRUE))
  =>
  (printout t "Loading navgraph feature" crlf)
  (ff-feature-request "navgraph")
)

(defrule executive-conditional-robot-memory-init
  "Load robot-memory feature required for PDDL."
  (executive-init)
  (confval (path "/clips-executive/use_pddl") (type BOOL) (value TRUE))
  =>
  (printout t "Robot Memory feature" crlf)
  (ff-feature-request "robot_memory")
)

(defrule executive-conditional-pddl-init
  "Load PDDL feature if requested in the config."
  (executive-init)
  (ff-feature-loaded blackboard)
  (confval (path "/clips-executive/use_pddl") (type BOOL) (value TRUE))
  (ff-feature-loaded robot_memory)
  =>
  (printout t "Loading PDDL Planner interface" crlf)
  (path-load "pddl-init.clp")
)

(defrule executive-init-stage2
	(executive-init)
	(ff-feature-loaded blackboard)
	=>

	; Here we load basic, non-flow components which deal with
	; representation and templates
	(path-load "plan.clp")
	(path-load "domain.clp")
	(path-load "worldmodel.clp")
	(path-load "wm-domain-sync.clp")
  (path-load "wm-config.clp")
	(path-load "blackboard-init.clp")
	(path-load "skills-init.clp")
)

(defrule executive-init-load-robot-memory-sync
  "Load the robot memory domain model synchronization if enabled in the config."
	(executive-init)
  (ff-feature-loaded robot_memory)
  ; Only load after the domain file has been loaded.
  (domain-loaded)
  (confval (path "/clips-executive/sync_domain_facts") (type BOOL) (value TRUE))
  =>
  (path-load "robot-memory-sync.clp")
)

(defrule executive-init-stage3
	(executive-init)
	(ff-feature-loaded skills)
  (or (ff-feature-loaded navgraph)
      (not (confval (path "/clips-executive/use_navgraph")
            (type BOOL) (value TRUE)))
  )
  (or (and (ff-feature-loaded pddl_planner) (ff-feature-loaded robot_memory))
      (not (confval (path "/clips-executive/use_pddl")
            (type BOOL) (value TRUE)))
  )
  (confval (path "/clips-executive/spec") (type STRING) (value ?spec))
	=>
	; Common spec config prefix
	(bind ?pf (str-cat "/clips-executive/specs/" ?spec "/"))
	(foreach ?component (create$ "domain" "worldmodel" "state-estimation"
                               "goal-reasoner" "goal-expander"
                               "macro-expansion" "action-selection"
                               "action-execution" "execution-monitoring")
		(do-for-fact ((?c confval)) (and (eq ?c:path (str-cat ?pf ?component)) (eq ?c:type STRING))
			(if ?c:is-list
			 then
			  (progn$ (?v ?c:list-value)
					(printout t "Loading component '" ?component "/" ?v-index "' (" ?v ")" crlf)
					(path-load ?v)
				)
			 else
				(printout t "Loading component '" ?component "' (" ?c:value ")" crlf)
				(path-load ?c:value)
			)
		)
	)
)
