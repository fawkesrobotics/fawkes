
(defrule goal-expander-create-sequence
	?g <- (goal (mode SELECTED) (id TESTGOAL))
	=>
	(assert
		(plan (id TESTGOAL-PLAN) (goal-id TESTGOAL))
		(plan-action (id 1) (plan-id TESTGOAL-PLAN) (duration 4.0)
		             (action-name say-hello)
		             (param-names name) (param-values "Peggy"))
		(plan-action (id 2) (plan-id TESTGOAL-PLAN) (duration 4.0)
		             (action-name print)
								 (param-names severity text) (param-values warn "This is a print test"))
		(plan-action (id 3) (plan-id TESTGOAL-PLAN) (duration 4.0)
		             (action-name say-goodbye))
		(plan-action (id 4) (plan-id TESTGOAL-PLAN) (duration 4.0)
		             (action-name say-hello-again)
		             (param-names name) (param-values "Peggy"))
		(plan-action (id 5) (plan-id TESTGOAL-PLAN) (duration 4.0)
		             (action-name say-cleanup))
	)
	(modify ?g (mode EXPANDED))
)
