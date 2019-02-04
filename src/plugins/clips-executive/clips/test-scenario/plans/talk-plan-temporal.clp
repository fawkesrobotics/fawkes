
(defrule plan-talk-expand
	?g <- (goal (mode SELECTED) (id ?goal-id) (class TALK))
	=>
	(bind ?plan-id (sym-cat ?goal-id -PLAN))
	(assert
		(plan (id ?plan-id) (goal-id ?goal-id))
		(plan-action (id 10) (goal-id ?goal-id) (plan-id ?plan-id)
                 (duration 4.0)
		             (action-name say-hello)
		             (param-names name) (param-values peggy))
		(plan-action (id 30) (goal-id ?goal-id) (plan-id ?plan-id)
		             (duration 4.0) (dispatch-time 15.0)
		             (action-name say-goodbye) (param-values peggy))
	)
)
