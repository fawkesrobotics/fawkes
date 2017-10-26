
(defrule goal-expander-create-sequence
	?g <- (goal (mode SELECTED) (id TESTGOAL))
	=>
	(assert
	 (plan (id TESTGOAL-PLAN) (goal-id TESTGOAL))
	 (plan-action (id 1) (plan-id TESTGOAL-PLAN) (duration 4.0)
								(action-name say-hello)
								(param-names name) (param-values "Peggy"))
	 (plan-action (id 2) (plan-id TESTGOAL-PLAN) (duration 4.0)
								(action-name say-goodbye))
	 )
	(modify ?g (mode EXPANDED))
)
