(deftemplate rl-goal-selection
  ; The ID of the goal which should be selected next.
  (slot goal-id (type SYMBOL))
  ; The ID of the goal selection message.
  (slot plan-id (type INTEGER))
  ; The current status of the rl goal selection.
  (slot status (type SYMBOL)
    (allowed-values
      PENDING RUNNING RL-FINISHED SELECTED
    )
  )
  ; TODO list of all executable goals
  (slot goal (type STRING))
)

(deftemplate rl-action-selection
	(slot next-action (type SYMBOL));(type STRING))
)

(deftemplate rl-waiting)

(defrule rl-waiting-assert
	(not (reset-domain-facts))
	(not (rl-waiting))
	?g <- (goal (mode FORMULATED))
	(not (goal (mode SELECTED)))
=>
	(printout t crlf "Assert rl waiting" ?g crlf crlf)
	(assert (rl-waiting))
	(rl-loop-start)
)

(defrule rl-waiting-retract
	?g<-(goal (mode SELECTED))
	?r <- (rl-waiting)
	=>
	(printout t crlf "Retracting rl waiting " ?r crlf "Goal: " ?g crlf crlf)
	(retract ?r)
)

; The rl calls the rl plugin and passes the goal id. It never leaves the DISPATCHED
; state.
(deffunction rl-call (?goal-id ?goal ) ;$?executableGoals)
	(printout info "Call the rl plugin." ?goal-id crlf ?goal crlf)
	;(bind ?interface (remote-if "Bla"))
	(bind ?m (blackboard-create-msg "RLAgentGoalSelectionInterface::goal-selection" "GSelectionMessage"))
	(blackboard-set-msg-field ?m "goal" ?goal-id)
	;(blackboard-set-msg-field ?m "exec goals" $?executableGoals)
	(printout info "Calling rl plugin for goal '" ?goal "' and " ?goal-id crlf) ;$?executableGoals crlf)
	(bind ?gen-id (blackboard-send-msg ?m))
	(assert (rl-goal-selection (goal-id ?goal-id) (goal ?goal) (plan-id ?gen-id) (status PENDING) ))

	;(assert (rl-plan (id ?goal-id) (goal ?goal) (status GEN-PENDING) (gen-id ?gen-id)))
)

;(defrule rl-clips-action-selection
;	?r <- (rl-action-selection (next-action ?a))
;	=>
;	(printout t crlf "in RL Plugin added fact: " ?r " with next action " ?a crlf crlf)
;	(retract ?r)
;)

(defrule rl-clips-action-selection
	?r <- (rl-action-selection (next-action ?a))
	?g <- (goal (id ?a) (mode ?m))
	=>
	(printout t crlf "in RL Plugin added fact: " ?r " with next action " ?a crlf )
	(printout t crlf "goal: " ?g "with in mode: "?m crlf crlf)
	(modify ?g (mode SELECTED))
	(retract ?r)
)


(defrule rl-clips-aspect
	?r <- (rl-init-test-fact)
	=>
	(printout t crlf "in RL Plugin added fact: " ?r crlf crlf)
	(retract ?r)
)

(defrule rl-goal-selection-select
  ;"Selects the goal with the given id"
  ?r <- (rl-goal-selection (goal-id ?goal-id) (status RL-FINISHED))
  (RLAgentGoalSelectionInterface (id "goal-selection") (next_select_goal ?n) (msg_id ?gen-id) (final TRUE) (success ?s))
  =>
  (printout t "The RL Plugin suggest the following goal for selection: " ?goal-id  " or " ?gen-id " success: " ?s crlf crlf)
  (printout t "Next goal: " ?n crlf crlf)
  (modify ?r (status SELECTED))
)

(defrule check-if-rl-goal-selection-running
  ;"Check whether the RL Plugin started."
  ?p <- (rl-goal-selection (status PENDING) (plan-id ?gen-id))
  (RLAgentGoalSelectionInterface (id "goal-selection") (msg_id ?gen-id))
  =>
  (printout t "RL goal selection started for ID " ?gen-id crlf)
  (modify ?p (status RUNNING))
)

(defrule check-if-rl-goal-selection-finished
  ;"Check whether the RL Plugin finished."
  ?p <- (rl-goal-selection (status RUNNING) (plan-id ?gen-id))
  (RLAgentGoalSelectionInterface (id "goal-selection") (msg_id ?gen-id) (final TRUE) (success ?s))
  =>
  (printout t "RL Plugin finished for ID " ?gen-id crlf)
  (modify ?p (status RL-FINISHED))
)


(defrule rl-goal-expand
	?g <- (goal (class RL) (id ?goal-id) (mode SELECTED))
	=>
	(printout t "Goal " ?goal-id " is expanded" crlf)
	(modify ?g (mode EXPANDED))
)

(defrule rl-goal-commit
	?g <- (goal (class RL) (mode EXPANDED))
	=>
	(printout t "In rl goal commit" crlf)
	(modify ?g (mode COMMITTED))
)

(defrule rl-goal-dispatch
	?g <- (goal (class RL) (id ?parent-id) (mode COMMITTED))
	;(goal (id ?sub-goal-id) (parent ?parent-id) (executable TRUE))
	; all executable goals with RL goal as "parent or grandfather"
	=>
	(printout t "In RL goal dispatch goal " ?parent-id crlf );" child " ?sub-goal-id crlf crlf)
	(do-for-fact ((?f goal))
		(eq ?f:parent ?parent-id)
			 ;(eq ?f:executable TRUE))
		(printout t "Executable child: " ?f " " ?f:id crlf crlf)
	)
	
	(rl-goal-selection-start (fact-slot-value ?g id) " TEST-STRING-RL ") ;calling RL Plugin via CLIPS Feature function
	;(rl-call ?parent-id ?g) ;calls RL Plugin via BB Interface - sending GSelection Message
	(modify ?g (mode DISPATCHED))
)

(defrule rl-goal-evaluate
	?g <- (goal (class RL) (mode FINISHED))
	=>
	(modify ?g (mode EVALUATED))
)
