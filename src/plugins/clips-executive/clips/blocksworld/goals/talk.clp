
; #  Commit to goal (we "intend" it)
(defrule talk-goal-commit
	?g <- (goal (id ?goal-id) (parent ?id) (class TALK|PRINT) (mode EXPANDED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	=>
	(modify ?g (mode COMMITTED) (committed-to ?plan-id))
)

; #  Dispatch goal (action selection and execution now kick in)
(defrule talk-goal-dispatch
	?g <- (goal (class TALK|PRINT) (mode COMMITTED)
							(required-resources $?req)
							(acquired-resources $?acq&:(subsetp ?req ?acq)))
	=>
	(modify ?g (mode DISPATCHED))
)

; #  Goal Monitoring
(defrule talk-goal-evaluate-completed
	?g <- (goal (id ?goal-id) (class TALK|PRINT) (mode FINISHED) (outcome COMPLETED))
	=>
	(printout t "Goal '" ?goal-id "' has been completed, evaluating" crlf)
	(modify ?g (mode EVALUATED))
)

(defrule talk-goal-evaluate-failed
	?g <- (goal (id ?goal-id) (class TALK|PRINT) (mode FINISHED)
	            (outcome ?outcome&FAILED|REJECTED))
	=>
	(printout t "Goal '" ?goal-id "' has failed (" ?outcome "), evaluating" crlf)
	(modify ?g (mode EVALUATED))
)
