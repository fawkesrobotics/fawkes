
; The HANG-NOOP does nothing. It never leaves the DISPATCHED
; state.

(defrule hang-noop-goal-expand
	?g <- (goal (class HANG-NOOP) (mode SELECTED))
	=>
	(modify ?g (mode EXPANDED))
)

(defrule hang-noop-goal-commit
	?g <- (goal (class HANG-NOOP) (mode EXPANDED))
	=>
	(modify ?g (mode COMMITTED))
)

(defrule hang-noop-goal-dispatch
	?g <- (goal (class HANG-NOOP) (mode COMMITTED))
	=>
	(modify ?g (mode DISPATCHED))
)

(defrule hang-noop-goal-evaluate
	?g <- (goal (class HANG-NOOP) (mode FINISHED))
	=>
	(modify ?g (mode EVALUATED))
)
