
; The ALWAYS-REJECT does only one thing, reject after expansion

(defrule always-reject-goal-expand
	?g <- (goal (class ALWAYS-REJECT) (mode SELECTED))
	=>
	(modify ?g (mode EXPANDED))
)

(defrule always-reject-goal-reject
	?g <- (goal (class ALWAYS-REJECT) (mode EXPANDED))
	=>
	(modify ?g (mode FINISHED) (outcome REJECTED))
)

(defrule always-reject-goal-evaluate
	?g <- (goal (class ALWAYS-REJECT) (mode FINISHED))
	=>
	(modify ?g (mode EVALUATED))
)
