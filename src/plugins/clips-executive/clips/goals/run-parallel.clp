;---------------------------------------------------------------------------
;  run-parallel.clp - CLIPS executive - goal to run all sub-goals in parallel
;
;  Created: Wed Dec 16 2020
;  Copyright  2020  Tarik Viehmann
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Sub-type: RUN-SUBGOALS-IN-PARALLEL
; Perform: all sub-goals in parallel
; Params: (params continue-on FAILED|REJECTED) to not stop SELECTION of
;         other goals when a sub-goal FAILED/is REJECTED
;         other values will be ignored
; Succeed: if all sub-goal succeeds
; Fail:    if at least one sub-goal fails
; Reject:  if any sub-goal is rejected but no sub-goal failed
;
; A RUN-PARALLEL parent goal will run all sub-goals in parallel by
; continuously SELECTING all sub-goals with the highest priority.
; If any goal fails, the parent fails. If any sub-goal is rejected,
; the parent is rejected. If all goals have been completed successfully,
; the parent goal succeeds.
; The RUN-PARALLEL goal can be configured to continue execution of other goals
; upon encountering a FAILED or REJECTED sub-goal by including
; 'continue-on FAILED' or 'continue-on REJECTED' in the goal params
; (continue-on FAILED implies continue-on REJECTED).

;
; Interactions:
; - User FORMULATES goal
; - User SELECTS goal
; - User EXPANDS goal, consisting of:
;   * create goals with parent ID equal the RUN-PARALLEL goal ID
;   * set RUN-PARALLEL goal mode to EXPANDED
; - Automatic: if no sub-goal formulated -> FAIL
; - Automatic: if all sub-goals rejected -> REJECT
; - Automatic: SELECT all formulated sub-goals with highest priority,
;              SELECT next batch of sub-goals once all
;              previously SELECTED sub-goals are DISPATCHED|FINISHED|RETRACTED.
; - User: handle sub-goal expansion, committing, dispatching, evaluating
; - Automatic: when sub-goal is EVALUATED, outcome determines parent goal:
;   * REJECTED or FAILED: Reject formulated sub-goals
;                         (unless configured otherwise),
;                         wait for completion of started sub-goals,
;                         message
;   * COMPLETED: wait for completion of other sub-goals
;   -> Sub-goal is RETRACTED
; - Automatic: once all sub-goals are RETRACTED, finish the parent goal:
;   * if a sub-goal FAILED: outcome FAILED
;   * else if a sub-goal was REJECTED: outcome REJECTED
;   * else (all sub-goals were COMPLETED): outcome COMPLETED
; - User: EVALUATE goal
; - User: RETRACT goal

(deffunction run-parallel-stop-execution (?params ?sub-goal-outcome)
	(return (and (or (eq ?sub-goal-outcome FAILED)
	                 (eq ?sub-goal-outcome REJECTED))
	             (not (member$ (create$ continue-on FAILED) ?params))
	             (not (member$ (create$ continue-on ?sub-goal-outcome) ?params))))
)

(defrule run-parallel-goal-expand-failed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-IN-PARALLEL)
	             (mode EXPANDED))
	(not (goal (type ACHIEVE) (parent ?id)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
	            (error NO-SUB-GOALS)
	            (message (str-cat "No sub-goal for RUN-PARALLEL goal '" ?id "'")))
)

(defrule run-parallel-goal-commit
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-IN-PARALLEL)
	             (mode EXPANDED))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(modify ?gf (mode COMMITTED))
)

(defrule run-parallel-goal-dispatch
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-IN-PARALLEL)
	             (mode COMMITTED)
	             (required-resources $?req)
	             (acquired-resources $?acq&:(subsetp ?req ?acq)))
	=>
	(modify ?gf (mode DISPATCHED))
)

(defrule run-parallel-subgoals-select
	(goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-IN-PARALLEL)
	      (mode DISPATCHED) (params $?params))
	(goal (parent ?id) (id ?sub-id) (type ACHIEVE) (mode FORMULATED)
	      (priority ?prio))
	(not (goal (parent ?id) (type ACHIEVE) (mode FORMULATED)
	           (priority ?o-prio&:(> ?o-prio ?prio))))
	(not (goal (parent ?id) (type ACHIEVE) (mode SELECTED|EXPANDED|COMMITTED)))
	(not (goal (parent ?id) (type ACHIEVE)
	           (outcome ?outcome&:(run-parallel-stop-execution ?params ?outcome))))
	=>
	(do-for-all-facts ((?g goal)) (and (eq ?g:parent ?id)
	                                   (eq ?g:mode FORMULATED)
	                                   (eq ?g:priority ?prio))
		(modify ?g (mode SELECTED))
	)
)

(defrule run-parallel-subgoal-reject-other-subgoal-rejected
	(goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-IN-PARALLEL)
	      (mode DISPATCHED) (params $?params))
	?sg <- (goal (parent ?id) (id ?sub-id) (type ACHIEVE) (mode FORMULATED)
	              (priority ?prio))
	(not (goal (parent ?id) (type ACHIEVE) (mode FORMULATED)
	           (priority ?o-prio&:(< ?o-prio ?prio))))
	(not (goal (parent ?id) (type ACHIEVE) (mode SELECTED|EXPANDED|COMMITTED)))
	(goal (parent ?id) (type ACHIEVE)
	      (outcome ?outcome&:(run-parallel-stop-execution ?params ?outcome)))
	=>
	(modify ?sg (mode FINISHED) (outcome REJECTED))
)

(defrule run-parallel-subgoal-evaluated
	(goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-IN-PARALLEL)
	      (mode DISPATCHED))
	?sg <- (goal (parent ?id) (type ACHIEVE) (mode EVALUATED))
	=>
	(modify ?sg (mode RETRACTED))
)

(defrule run-parallel-goal-finish-all-subgoals-retracted
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-IN-PARALLEL)
	             (mode DISPATCHED))
	(not (goal (parent ?id) (type ACHIEVE)
	           (acquired-resources $?acq&:(> (length ?acq) 0))))
	(not (goal (parent ?id) (type ACHIEVE) (mode ~RETRACTED)))
	=>
	(if (not (do-for-fact ((?g goal))
	                      (and (eq ?g:parent ?id) (eq ?g:outcome FAILED))
		(modify ?gf (mode FINISHED) (outcome ?g:outcome)
		          (error SUB-GOAL-FAILED ?g:id)
		          (message (str-cat "Sub-goal '" ?g:id "' of RUN-PARALLEL goal '"
		                            ?id "' has failed")))))
	 then
		(if (not (do-for-fact ((?g goal))
		                    (and (eq ?g:parent ?id) (eq ?g:outcome REJECTED))
			(modify ?gf (mode FINISHED) (outcome ?g:outcome)
			          (error SUB-GOAL-REJECTED ?g:id)
			          (message (str-cat "Sub-goal '" ?g:id "' of RUN-PARALLEL goal '"
			                            ?id "' was rejected")))))
		 then
			(modify ?gf (mode FINISHED) (outcome COMPLETED))
		)
	)
)
