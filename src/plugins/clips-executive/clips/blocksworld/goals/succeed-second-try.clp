
; The SUCCEED-SECOND-TRY fails on the first try, and succeeds
; on the second. It keeps number of tries in meta slot.

(defrule succeed-second-try-goal-expand
	?g <- (goal (class SUCCEED-SECOND-TRY) (mode SELECTED))
	=>
	(modify ?g (mode EXPANDED))
)

(defrule succeed-second-try-goal-commit
	?g <- (goal (class SUCCEED-SECOND-TRY) (mode EXPANDED))
	=>
	(modify ?g (mode COMMITTED))
)

(defrule succeed-second-try-goal-dispatch
	?g <- (goal (class SUCCEED-SECOND-TRY) (mode COMMITTED))
	=>
	(modify ?g (mode DISPATCHED))
)

(defrule succeed-second-try-goal-fail
	?g <- (goal (class SUCCEED-SECOND-TRY) (mode DISPATCHED) (meta))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED) (meta tried-once))
)

(defrule succeed-second-try-goal-succeed
	?g <- (goal (class SUCCEED-SECOND-TRY) (mode DISPATCHED) (meta tried-once))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule succeed-second-try-goal-evaluate
	?g <- (goal (class SUCCEED-SECOND-TRY) (mode FINISHED))
	=>
	(modify ?g (mode EVALUATED))
)
