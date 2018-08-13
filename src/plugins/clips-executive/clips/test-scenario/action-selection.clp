
(defrule action-selection-select
	(goal (id ?goal-id) (mode DISPATCHED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (state FORMULATED)
											(action-name ?action-name))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state ~FORMULATED&~FINAL)))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FORMULATED) (id ?oid&:(< ?oid ?id))))
	=>
	(modify ?pa (state PENDING))
)

(defrule action-selection-done
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state ~FINAL)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FAILED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)

