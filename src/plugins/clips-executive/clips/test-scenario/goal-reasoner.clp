
; #  Goal Creation
(defrule goal-reasoner-parent-create
	(domain-loaded)
	(not (goal))
	(not (test-performed))
  (domain-facts-loaded)
	=>
	(bind ?goal-id (sym-cat TEST-PARENT- (gensym*)))
	(assert (goal (id ?goal-id) (class TESTGOAL-PARENT)
								;(sub-type RUN-ONE-OF-SUBGOALS)))
								;(sub-type RUN-ALL-OF-SUBGOALS)))
								(sub-type TRY-ALL-OF-SUBGOALS)))
								;(sub-type RETRY-SUBGOAL) (params max-tries 3))
					;(goal (id (sym-cat TEST-PARENT- (gensym*))) (parent ?goal-id) (class TESTGOAL)))

	; This is just to make sure we formulate the goal only once.
	; In an actual domain this would be more sophisticated.
	(assert (test-performed))
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
								(priority 10)); (required-resources FOO BAR) (acquired-resources FOO BAR))
					(goal (id (sym-cat TESTGOAL- (gensym*))) (parent ?goal-id) (class TESTGOAL)
								(priority 20)))
	(modify ?g (mode EXPANDED))
)

; #  Commit to goal (we "intend" it)
; A goal might actually be expanded into multiple plans, e.g., by
; different planners. This step would allow to commit one out of these
; plans.
(defrule goal-reasoner-subgoal-commit
	;?pg <- (goal (id ?id) (meta num-tries ?num-tries))
	?g <- (goal (parent ?id) (class TESTGOAL) (mode EXPANDED) (priority 10))
	=>
	(modify ?g (mode COMMITTED))
	;(modify ?g (mode FINISHED) (outcome FAILED))
	; (if (> ?num-tries 1)
	; then
	; 	(modify ?g (mode FINISHED) (outcome FAILED))
	; else
	; 	(modify ?g (mode FINISHED) (outcome REJECTED))
	; )
)
(defrule goal-reasoner-subgoal-reject
	?g <- (goal (class TESTGOAL) (mode EXPANDED) (priority 20))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)

; #  Dispatch goal (action selection and execution now kick in)
; Trigger execution of a plan. We may commit to multiple plans
; (for different goals), e.g., one per robot, or for multiple
; orders. It is then up to action selection and execution to determine
; what to do when.
(defrule goal-reasoner-subgoal-dispatch
	?g <- (goal (class TESTGOAL) (mode COMMITTED)
							(required-resources $?req)
							(acquired-resources $?acq&:(subsetp ?req ?acq)))
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
	?g <- (goal (id ?goal-id) (class TESTGOAL) (mode FINISHED) (outcome FAILED|REJECTED))
	=>
	(printout t "Goal '" ?goal-id "' has failed, evaluating" crlf)
	(modify ?g (mode EVALUATED))
)

; # Parent Goal evaluation
(defrule goal-reasoner-goal-evaluate
	?g <- (goal (id ?goal-id) (class TESTGOAL-PARENT) (mode FINISHED))
	=>
  (modify ?g (mode EVALUATED))
)

; # Parent Goal Clean up
(defrule goal-reasoner-goal-cleanup-completed
	?g <- (goal (id ?goal-id) (class TESTGOAL-PARENT) (mode EVALUATED) (outcome COMPLETED))
	=>
  (modify ?g (mode RETRACTED))
)

(defrule goal-reasoner-goal-do-not-cleanup-failed
	?g <- (goal (id ?goal-id) (class TESTGOAL-PARENT) (mode EVALUATED) (outcome FAILED|REJECTED)
							(message ?message))
	=>
  (modify ?g (message (str-cat ?message "  NOT cleaning up because failed, test case")))
)
