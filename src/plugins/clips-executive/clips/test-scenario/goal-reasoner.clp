
(deftemplate goal-meta
	(slot goal-id (type SYMBOL))
	(slot num-tries (type INTEGER))
)

(defglobal
	?*GOAL-MAX-TRIES* = 3
)

; #  Goal Creation
(defrule goal-reasoner-create
	(domain-loaded)
	(not (goal))
	(not (goal-already-tried))
  (domain-facts-loaded)
	=>
	(assert (goal (id (sym-cat TESTGOAL- (gensym*))) (class TESTGOAL)))
	; This is just to make sure we formulate the goal only once.
	; In an actual domain this would be more sophisticated.
	(assert (goal-already-tried))
)


; #  Goal Selection
; We can choose one or more goals for expansion, e.g., calling
; a planner to determine the required steps.
(defrule goal-reasoner-select
	?g <- (goal (id ?goal-id) (class TESTGOAL) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
	(assert (goal-meta (goal-id ?goal-id)))
)

; #  Commit to goal (we "intend" it)
; A goal might actually be expanded into multiple plans, e.g., by
; different planners. This step would allow to commit one out of these
; plans.
(defrule goal-reasoner-commit
	?g <- (goal (class TESTGOAL) (mode EXPANDED))
	=>
	(modify ?g (mode COMMITTED))
)

; #  Dispatch goal (action selection and execution now kick in)
; Trigger execution of a plan. We may commit to multiple plans
; (for different goals), e.g., one per robot, or for multiple
; orders. It is then up to action selection and execution to determine
; what to do when.
(defrule goal-reasoner-dispatch
	?g <- (goal (class TESTGOAL) (mode COMMITTED))
	=>
	(modify ?g (mode DISPATCHED))
)

; #  Goal Monitoring
(defrule goal-reasoner-evaluate-completed
	?g <- (goal (id ?goal-id) (class TESTGOAL) (mode FINISHED) (outcome COMPLETED))
	?gm <- (goal-meta (goal-id ?goal-id))
	=>
	(printout t "Goal '" ?goal-id "' has been completed, evaluating" crlf)
	(modify ?g (mode EVALUATED))
)

(defrule goal-reasoner-evaluate-failed
	?g <- (goal (id ?goal-id) (class TESTGOAL) (mode FINISHED) (outcome FAILED))
	?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries))
	=>
	(printout t "Goal '" ?goal-id "' has failed, evaluating" crlf)
	(bind ?num-tries (+ ?num-tries 1))
	(modify ?gm (num-tries ?num-tries))
	(modify ?g (mode EVALUATED))
)

; # Goal Clean up
(defrule goal-reasoner-cleanup-completed
	?g <- (goal (id ?goal-id) (class TESTGOAL) (mode EVALUATED) (outcome COMPLETED))
	?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries))
	=>
	(printout t "Goal '" ?goal-id "' has been Evaluated, cleaning up" crlf)
	(delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
		(delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
			(retract ?a)
		)
		(retract ?g ?gm)
	)
)

(defrule goal-reasoner-cleanup-failed
  ?g <- (goal (id ?goal-id) (class TESTGOAL) (mode EVALUATED) (outcome FAILED))
  ?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries))
  =>
  (printout t "Goal '" ?goal-id "' has been Evaluated, cleaning up" crlf)
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
    (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
      (retract ?a)
    )
    (retract ?p)
  )
  (if (< ?num-tries ?*GOAL-MAX-TRIES*)
	then
		(printout t "Triggering re-expansion" crlf)
		(modify ?g (mode SELECTED))
	else
		(printout t "Goal failed " ?num-tries " times, aborting" crlf)
		(retract ?g ?gm)
	)
)
