(deftemplate rl-finished-goal
	(slot goal-id (type SYMBOL));
	(slot outcome (type SYMBOL));
	(slot result (type INTEGER));
)

(defrule tower-one-executable-check-true
	?g <- (goal (id ?goal-id) (class TOWER-C1) (mode FORMULATED) (is-executable ?executable&FALSE)
			    (params buttom ?b
						top ?t))
	(domain-fact (name ontable) (param-values ?b))
	(domain-fact (name ontable) (param-values ?t))
	=>
	(printout t "Goal '" ?goal-id "' is executable" crlf)
	(modify ?g (is-executable TRUE))
)
(defrule tower-one-executable-check-false
	?g <- (goal (id ?goal-id) (class TOWER-C1) (mode FORMULATED) (is-executable ?executable&TRUE)
			    (params buttom ?b
						top ?t))
	(not (domain-fact (name ontable) (param-values ?b)))
	(not (domain-fact (name ontable) (param-values ?t)))
	=>
	(printout t "Goal '" ?goal-id "' is NOT executable" crlf)
	(modify ?g (is-executable FALSE))
)

(defrule tower-two-executable-check-true
	?g <- (goal (id ?goal-id) (class TOWER-C2) (mode FORMULATED) (is-executable ?executable&FALSE)
			    (params buttom ?b
						middle ?m
						top ?t))
	
	(domain-fact (name ontable) (param-values ?b))
	(domain-fact (name ontable) (param-values ?m))
	(domain-fact (name ontable) (param-values ?t))
	=>
	(printout t "Goal '" ?goal-id "' is executable" crlf)
	(modify ?g (is-executable TRUE))
)
(defrule tower-two-executable-check-false
	?g <- (goal (id ?goal-id) (class TOWER-C2) (mode FORMULATED) (is-executable ?executable&TRUE)
			    (params buttom ?b
						middle ?m
						top ?t))

	(not (domain-fact (name ontable) (param-values ?b)))
	(not (domain-fact (name ontable) (param-values ?m)))
	(not (domain-fact (name ontable) (param-values ?t)))
	=>
	(printout t "Goal '" ?goal-id "' is NOT executable" crlf)
	(modify ?g (is-executable FALSE))
)

; #  Commit to goal (we "intend" it)
(defrule blocks-goal-commit
	?g <- (goal (id ?goal-id) (parent ?id) (class BLOCKS|TOWER-C1|TOWER-C2) (mode EXPANDED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	=>
	(printout t "Goal " ?goal-id " is expanded" crlf)
	(modify ?g (mode COMMITTED) (committed-to ?plan-id))
)

; #  Dispatch goal (action selection and execution now kick in)
(defrule blocks-goal-dispatch
	?g <- (goal (class BLOCKS|TOWER-C1|TOWER-C2) (id ?goal-id) (mode COMMITTED)
							(required-resources $?req)
							(acquired-resources $?acq&:(subsetp ?req ?acq)))
	=>
	(printout t "Goal '" ?goal-id "' is dispatched" crlf)
	(modify ?g (mode DISPATCHED))
)

; #  Goal Monitoring
(defrule blocks-goal-evaluate-completed
	?g <- (goal (id ?goal-id) (class BLOCKS|TOWER-C1) (mode FINISHED) (outcome COMPLETED))
	=>
	(printout t "Goal '" ?goal-id "' has been completed, evaluating" crlf)
	(assert (rl-finished-goal (goal-id ?goal-id) (outcome COMPLETED) (result 1)))
	(modify ?g (mode EVALUATED))
;	(assert (rl-waiting))

;	(printout t "asserted rl-waiting" crlf )
)

(defrule tower-two-goal-evaluate-completed
	?g <- (goal (id ?goal-id) (class TOWER-C2) (mode FINISHED) (outcome COMPLETED))
	=>
	(printout t "Goal '" ?goal-id "' has been completed, evaluating" crlf)
	(assert (rl-finished-goal (goal-id ?goal-id) (outcome COMPLETED) (result 5)))
	(modify ?g (mode EVALUATED))
)

(defrule blocks-goal-evaluate-failed
	?g <- (goal (id ?goal-id) (class BLOCKS|TOWER-C1|TOWER-C2) (mode FINISHED)
	            (outcome ?outcome&FAILED|REJECTED))
	=>
	(printout t "Goal '" ?goal-id "' has failed (" ?outcome "), evaluating" crlf)
	(assert (rl-finished-goal (goal-id ?goal-id) (outcome ?outcome) (result 0)))
	(modify ?g (mode EVALUATED))
)
