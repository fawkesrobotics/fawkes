
(defrule action-selection-select
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (state FORMULATED)
											(action-name ?action-name))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	(not (plan-action (state ~FORMULATED&~FINAL)))
	(not (plan-action (state FORMULATED) (id ?oid&:(< ?oid ?id))))
	=>
	(modify ?pa (state PENDING))
)

(defrule action-selection-done
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(not (plan-action (plan-id ?plan-id) (state ~FINAL)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(plan-action (state FAILED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)
