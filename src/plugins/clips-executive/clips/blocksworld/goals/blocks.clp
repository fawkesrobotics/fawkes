
; #  Commit to goal (we "intend" it)
(defrule blocks-goal-commit
	?g <- (goal (id ?goal-id) (parent ?id) (class BLOCKS|TOWER-C1|TOWER-C2) (mode EXPANDED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	=>
	(printout t "Goal " ?goal-id " is expanded" crlf)
	(modify ?g (mode COMMITTED) (committed-to ?plan-id))
)

; #  Dispatch goal (action selection and execution now kick in)
(defrule blocks-goal-dispatch
	?g <- (goal (class BLOCKS|TOWER-C1|TOWER-C2) (id ?goal-id) (mode COMMITTED)
							(required-resources $?req)
							(acquired-resources $?acq&:(subsetp ?req ?acq)))
	=>
	(printout t "Goal '" ?goal-id "' is dispatched" crlf)
	(modify ?g (mode DISPATCHED))
)

; #  Goal Monitoring
(defrule blocks-goal-evaluate-completed
	?g <- (goal (id ?goal-id) (class BLOCKS|TOWER-C1|TOWER-C2) (mode FINISHED) (outcome COMPLETED))
	=>
	(printout t "Goal '" ?goal-id "' has been completed, evaluating" crlf)
	(modify ?g (mode EVALUATED))
)

(defrule blocks-goal-evaluate-failed
	?g <- (goal (id ?goal-id) (class BLOCKS|TOWER-C1|TOWER-C2) (mode FINISHED)
	            (outcome ?outcome&FAILED|REJECTED))
	=>
	(printout t "Goal '" ?goal-id "' has failed (" ?outcome "), evaluating" crlf)
	(modify ?g (mode EVALUATED))
)
