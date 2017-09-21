
(defmodule SPEC (import MAIN ?ALL) (import PLAN ?ALL))

(defrule SPEC::spec-start
	(start)
	=>
	(printout t "Spec starting" crlf)
	(assert (plan-action (id 1) (plan-name foo) (duration 4.0)
											 (action-name say) (params text "Hello world" wait true)))
	(assert (plan-action (id 2) (plan-name foo) (duration 4.0)
											 (action-name say) (params text "Good bye" wait true)))
)

(defrule SPEC::spec-done
	(plan-action (id ?id) (status ?s))
	=>
	(printout t "Action " ?id " is now " ?s crlf)
)
