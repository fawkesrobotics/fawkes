
; The ALWAYS-FAIL does only one thing, fail after dispatch

(defrule always-fail-goal-expand
	?g <- (goal (class ALWAYS-FAIL) (mode SELECTED))
	=>
	(modify ?g (mode EXPANDED))
)

(defrule always-fail-goal-commit
	?g <- (goal (class ALWAYS-FAIL) (mode EXPANDED))
	=>
	(modify ?g (mode COMMITTED))
)

(defrule always-fail-goal-dispatch
	?g <- (goal (class ALWAYS-FAIL) (mode COMMITTED))
	=>
	(modify ?g (mode DISPATCHED))
)

(defrule always-fail-goal-fail
	?g <- (goal (class ALWAYS-FAIL) (mode DISPATCHED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)

(defrule always-fail-goal-evaluate
	?g <- (goal (class ALWAYS-FAIL) (mode FINISHED))
	=>
	(modify ?g (mode EVALUATED))
)
