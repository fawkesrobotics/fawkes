
(defrule spec-init
	(executive-init)
	(ff-feature-loaded skills)
	=>
	(path-load "plan.clp")
	(path-load "plan-exec.clp")
	(path-load "exectest.clp")
)
