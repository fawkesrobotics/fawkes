
(defrule goal-expander-create-sequence
	?g <- (goal (mode SELECTED) (id TESTGOAL))
	=>
	(assert (plan (id TESTGOAL-PLAN) (goal-id TESTGOAL)))
	(assert (plan-action (id 1) (plan-id TESTGOAL-PLAN) (duration 4.0)
											 (action-name say) (params text wait)
                       (param-values "Hello world" true)))
	(assert (plan-action (id 2) (plan-id TESTGOAL-PLAN) (duration 4.0)
											 (action-name say) (params textFAIL)
                       (param-values "Good bye")))
	(modify ?g (mode EXPANDED))
)
