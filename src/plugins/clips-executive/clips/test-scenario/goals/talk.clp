
; #  Commit to goal (we "intend" it)
(defrule talk-goal-commit
	?g <- (goal (parent ?id) (class TALK) (mode EXPANDED))
	=>
	(modify ?g (mode COMMITTED))
)

; #  Dispatch goal (action selection and execution now kick in)
(defrule talk-goal-dispatch
	?g <- (goal (class TALK) (mode COMMITTED)
							(required-resources $?req)
							(acquired-resources $?acq&:(subsetp ?req ?acq)))
	=>
	(modify ?g (mode DISPATCHED))
)

; #  Goal Monitoring
(defrule talk-goal-evaluate-completed
	?g <- (goal (id ?goal-id) (class TALK) (mode FINISHED) (outcome COMPLETED))
	=>
	(printout t "Goal '" ?goal-id "' has been completed, evaluating" crlf)
	(modify ?g (mode EVALUATED))
)

(defrule talk-goal-evaluate-failed
	?g <- (goal (id ?goal-id) (class TALK) (mode FINISHED) (outcome FAILED|REJECTED))
	=>
	(printout t "Goal '" ?goal-id "' has failed, evaluating" crlf)
	(modify ?g (mode EVALUATED))
)
