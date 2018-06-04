
(defrule action-selection-select
	(goal (id ?goal-id) (mode DISPATCHED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (status FORMULATED)
											(action-name ?action-name))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (status ~FORMULATED&~FINAL)))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (status FORMULATED) (id ?oid&:(< ?oid ?id))))
	=>
	(modify ?pa (status PENDING))
)

(defrule action-selection-done
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (status ~FINAL)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(plan-action (goal-id ?goal-id) (plan-id ?plan-id) (status FAILED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)

