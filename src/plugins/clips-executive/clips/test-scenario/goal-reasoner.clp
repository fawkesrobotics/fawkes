
(defglobal
	?*GOAL-MAX-TRIES* = 3
)

; #  Goal Creation
(defrule goal-reasoner-parent-create
	(domain-loaded)
	(not (goal))
	(not (goal-already-tried))
  (domain-facts-loaded)
	=>
	(assert (goal (id (sym-cat TEST-PARENT- (gensym*)))
								(sub-type RUN-ONE-OF-SUBGOALS) (class TESTGOAL-PARENT)))
	; This is just to make sure we formulate the goal only once.
	; In an actual domain this would be more sophisticated.
	(assert (goal-already-tried))
)


; #  Goal Selection
; We can choose one or more goals for expansion, e.g., calling
; a planner to determine the required steps.
(defrule goal-reasoner-parent-select
	?g <- (goal (id ?goal-id) (class TESTGOAL-PARENT) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
)

; #  Parent Goal Expansion
; We can choose one or more goals for expansion, e.g., calling
; a planner to determine the required steps.
(defrule goal-reasoner-parent-expand
	?g <- (goal (id ?goal-id) (class TESTGOAL-PARENT) (mode SELECTED))
	=>
	(printout t "Expanding " ?goal-id crlf)
	(assert	(goal (id (sym-cat TESTGOAL- (gensym*))) (parent ?goal-id) (class TESTGOAL)
								(priority 10) (meta num-tries 0))
					(goal (id (sym-cat TESTGOAL- (gensym*))) (parent ?goal-id) (class TESTGOAL)
								(priority 20) (meta num-tries 0)))
	(modify ?g (mode EXPANDED))
)

; #  Commit to goal (we "intend" it)
; A goal might actually be expanded into multiple plans, e.g., by
; different planners. This step would allow to commit one out of these
; plans.
(defrule goal-reasoner-subgoal-commit
	?g <- (goal (class TESTGOAL) (mode EXPANDED) (priority 10))
	=>
	(modify ?g (mode COMMITTED))
)
(defrule goal-reasoner-subgoal-reject
	?g <- (goal (class TESTGOAL) (mode EXPANDED) (priority 20))
	=>
	(modify ?g (mode FINISHED) (outcome REJECTED))
)

; #  Dispatch goal (action selection and execution now kick in)
; Trigger execution of a plan. We may commit to multiple plans
; (for different goals), e.g., one per robot, or for multiple
; orders. It is then up to action selection and execution to determine
; what to do when.
(defrule goal-reasoner-subgoal-dispatch
	?g <- (goal (class TESTGOAL) (mode COMMITTED))
							;(required-resources $?req)
							;(acquired-resources $?acq&:(subsetp ?req ?acq)))
	=>
	(modify ?g (mode DISPATCHED))
)

; #  Goal Monitoring
(defrule goal-reasoner-subgoal-evaluate-completed
	?g <- (goal (id ?goal-id) (class TESTGOAL) (mode FINISHED) (outcome COMPLETED))
	=>
	(printout t "Goal '" ?goal-id "' has been completed, evaluating" crlf)
	(modify ?g (mode EVALUATED))
)

(defrule goal-reasoner-subgoal-evaluate-failed
	?g <- (goal (id ?goal-id) (class TESTGOAL) (mode FINISHED) (outcome FAILED) (meta num-tries ?num-tries))
	=>
	(printout t "Goal '" ?goal-id "' has failed, evaluating" crlf)
	(bind ?num-tries (+ ?num-tries 1))
	(modify ?g (mode EVALUATED) (meta num-tries ?num-tries))
)

; ; # Goal Clean up
(defrule goal-reasoner-subgoal-cleanup-completed
	?g <- (goal (id ?goal-id) (class TESTGOAL) (mode EVALUATED) (outcome COMPLETED))
	=>
	(printout t "Goal '" ?goal-id "' has been Evaluated, cleaning up" crlf)
	(delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
		(delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
			(retract ?a)
		)
	)
  (modify ?g (mode RETRACTED))
)

(defrule goal-reasoner-subgoal-cleanup-failed
  ?g <- (goal (id ?goal-id) (class TESTGOAL) (mode EVALUATED) (outcome FAILED) (meta num-tries ?num-tries))
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
    (modify ?g (mode RETRACTED))
	)
)

(defrule goal-reasoner-retract-goal
  ?g <- (goal (mode RETRACTED) (acquired-resources))
  =>
  (retract ?g)
)
