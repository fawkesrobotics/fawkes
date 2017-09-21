
(defrule goal-reasoner-create-goal
	(not (goal))
	=>
	(assert (goal (id TESTGOAL)))
)

(defrule goal-reasoner-select-goal
	?g <- (goal (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
)
