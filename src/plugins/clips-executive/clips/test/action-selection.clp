
(defrule action-select-select
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status FORMULATED)
											(action-name ?action-name) (params $?params))
	(not (plan-action (status PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (status FORMULATED) (id ?oid&:(< ?oid ?id))))
	=>
	(modify ?pa (status PENDING))
)
