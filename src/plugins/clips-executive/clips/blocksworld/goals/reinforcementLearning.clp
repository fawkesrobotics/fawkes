; Defining templates and rules for goal selection with rl

(deftemplate rl-goal-selection
	(slot next-goal-id (type SYMBOL))
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
	;(rl-loop-start)
)

(defrule rl-waiting-retract
	?g<-(goal (mode SELECTED))
	?r <- (rl-waiting)
	=>
	(printout t crlf "Retracting rl waiting " ?r crlf "Goal: " ?g crlf crlf)
	(retract ?r)
)


;TODO Check for mode if mode is FORMULATED then leaf it
;TODO add function for mode EVALUATED or rejected to assert:
;(printout t "Goal '" ?goal-id "' has failed (" ?outcome "), evaluating" crlf)
;(assert (rl-finished-goal (goal-id ?goal-id) (outcome ?outcome) (result 0)))
(defrule rl-clips-goal-selection
	?r <- (rl-goal-selection (next-goal-id ?a))
	?g <- (goal (id ?a) (mode ?m))
	=>
	(printout t crlf "in RL Plugin added fact: " ?r " with next action " ?a crlf )
	(printout t crlf "goal: " ?g "with in mode: "?m crlf crlf)
	(modify ?g (mode SELECTED))
	(retract ?r)
)



(defrule rl-goal-expand
	?g <- (goal (class RL) (id ?goal-id) (mode SELECTED))
	=>
	(printout t "In RL goal expand " ?goal-id crlf)
	(modify ?g (mode EXPANDED))
)

(defrule rl-goal-commit
	?g <- (goal (class RL) (id ?goal-id) (mode EXPANDED))
	=>
	(printout t "In RL goal commit" ?goal-id crlf)
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
	?g <- (goal (class RL)  (id ?goal-id) (mode FINISHED))
	=>
	(printout t "In RL goal evaluate " ?goal-id crlf)
	(modify ?g (mode EVALUATED))
)
