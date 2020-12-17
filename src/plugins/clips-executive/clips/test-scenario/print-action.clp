
(defrule print-action-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (state PENDING)
                      (action-name print) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
	=>
	(bind ?severity (plan-action-arg severity ?param-names ?param-values info))
	(bind ?text     (plan-action-arg text ?param-names ?param-values ""))
	(printout ?severity ?text crlf)
	(modify ?pa (state RUNNING))
)

(defrule print-action-end
	(declare (salience ?*SALIENCE-LOW*))
	?pa <- (plan-action (state RUNNING) (action-name print))
	=>
	(modify ?pa (state EXECUTION-SUCCEEDED))
)
