
; #  Goal Creation
(defrule goal-reasoner-parent-create
	(domain-loaded)
	(not (goal))
	(not (test-performed))
  (domain-facts-loaded)
  (skiller-control (acquired TRUE))
	=>
	; (bind ?goal-id (sym-cat TEST-PARENT- (gensym*)))
	; (assert (goal (id ?goal-id) (class TESTGOAL-PARENT)
	; 							;(sub-type RUN-ONE-OF-SUBGOALS)))
	; 							;(sub-type RUN-ALL-OF-SUBGOALS)))
	; 							;(sub-type TRY-ALL-OF-SUBGOALS)))
	; 							(sub-type TIMEOUT-SUBGOAL) (params timeout 3.0)))
	; 							;(sub-type RETRY-SUBGOAL) (params max-tries 3))
	; 				;(goal (id (sym-cat TEST-PARENT- (gensym*))) (parent ?goal-id) (class TESTGOAL)))

	(goal-tree-assert-run-one TESTGOAL-PARENT
	 (assert (goal (id (gensym*)) (class ALWAYS-REJECT)))
	 (goal-tree-assert-run-all AUTOMATIC-SUBGOAL
		(goal-tree-assert-run-parallel AUTOMATIC-SUBGOAL NONE
		 (assert (goal (id (gensym*)) (class TALK)))
		 (goal-tree-assert-retry AUTOMATIC-SUBGOAL 3
		  (assert (goal (id (gensym*)) (class SUCCEED-SECOND-TRY))))
		)
		(goal-tree-assert-try-all AUTOMATIC-SUBGOAL
		 (goal-tree-assert-timeout AUTOMATIC-SUBGOAL 5.0
			(assert (goal (id (gensym*)) (class HANG-NOOP)))
		 )
		 (goal-tree-assert-run-parallel-delayed AUTOMATIC-SUBGOAL FAILED
			(assert (goal (id (gensym*)) (class ALWAYS-REJECT)))
			(goal-tree-assert-run-parallel AUTOMATIC-SUBGOAL NONE
			 (assert (goal (id (sym-cat PARALLEL-PRINT- (gensym*))) (class PRINT)))
			 (assert (goal (id (sym-cat PARALLEL-PRINT- (gensym*))) (class PRINT)))
			)
		 )
		 (goal-tree-assert-run-parallel-delayed AUTOMATIC-SUBGOAL REJECTED
			(assert (goal (id (gensym*)) (class ALWAYS-FAIL)))
			(assert (goal (id (sym-cat NEVER-EXPANDED- (gensym*))) (class PRINT)))
		 )
		 (assert (goal (id (gensym*)) (class ALWAYS-FAIL)))
		 (assert (goal (id (sym-cat FINALLY-SUCCEED- (gensym*))) (class PRINT)))
		)
	 )
	)

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
(defrule goal-reasoner-parent-expand
	?g <- (goal (id ?goal-id) (class TESTGOAL-PARENT) (mode SELECTED))
	=>
	(modify ?g (mode EXPANDED))
)

; # Parent Goal evaluation
(defrule goal-reasoner-goal-evaluate
	?g <- (goal (id ?goal-id) (class TESTGOAL-PARENT) (mode FINISHED))
	=>
  (modify ?g (mode EVALUATED))
)

; # Parent Goal Clean up
(defrule goal-reasoner-goal-cleanup-completed
	?g <- (goal (id ?goal-id) (class TESTGOAL-PARENT) (mode EVALUATED))
	=>
  (modify ?g (mode RETRACTED))
)

(defrule goal-reasoner-expanded
	?g <- (goal (id ?goal-id) (class TALK|PRINT) (mode SELECTED))
	(plan (goal-id ?goal-id))
	=>
  (modify ?g (mode EXPANDED))
)
