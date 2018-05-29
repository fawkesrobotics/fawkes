
(defrule print-action-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status PENDING)
                      (action-name print) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
	=>
	(bind ?severity (plan-action-arg severity ?param-names ?param-values info))
	(bind ?text     (plan-action-arg text ?param-names ?param-values ""))
	(printout ?severity ?text crlf)
	(modify ?pa (status EXECUTION-SUCCEEDED))
)
