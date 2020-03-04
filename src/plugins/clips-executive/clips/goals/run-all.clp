;---------------------------------------------------------------------------
;  run-all.clp - CLIPS executive - goal to run all sub-goals to completion
;
;  Created: Mon Jun 04 15:00:20 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Sub-type: RUN-ALL-OF-SUBGOALS
; Perform: one goal at a time, ordered by goal priority
; Succeed: if all sub-goal succeeds
; Fail:    if exactly one sub-goal fails
; Reject:  if any sub-goals is rejected
;
; A RUN-ALL parent goal will order the goals by priority and then
; start performing them in order. If any goal fails, the parent
; fails. If any sub-goal is rejected, the parent is rejected. If all
; goals have been completed successfully, the parent goal succeeds.
;
; Interactions:
; - User: FORMULATES goal
; - User: SELECTS goal
;   On SELECTED goal
;    * User FORMULATES sub-goals with parent ID equal the RUN-ALL goal ID
;                                                     (populating the children)
;    * Automatic: FAIL goal: if no sub-goal formulated
;    * Automatic: REJECT goal: if all sub-goals rejected
;    * USER: SELECTS sub-goal(s)
; - Automatic: if all selected sub-goal EXPANDED   -> goal EXPANDED
;                                                         (bottom up expantion)
; - Automatic: if root with all sub-goals expanded -> goal COMMITTED
; - Automatic: if parent committed                 -> goal COMMITTED
;         (top down committment to one or more goals with the hightst priority)
; - Automatic: if a sub-goal DISPATCHED            -> goal  DISPATCHED
;                                                       (bottom up DISPATCHING)
; - User: handle sub-goal expansion, committing, dispatching, evaluating
; - Automatic: when sub-goal is EVALUATED, outcome determines parent goal:
;   * REJECTED: mode FINISHED, outcome REJECTED, message
;   * FAILED: mode FINISHED, outcome FAILED, message
;   * COMPLETED: mode FINISHED, outcome COMPLETED
;   -> Sub-goal is RETRACTED.
; User: EVALUATE goal
; User: RETRACT goal



(defrule run-all-goal-expand-failed
     (declare (salience ?*SALIENCE-LOW*))
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS)
							 (mode SELECTED))
	(not (goal (type ACHIEVE) (parent ?id) (mode FORMULATED|SELECTED|EXPANDED)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
					(error NO-SUB-GOALS)
					(message (str-cat "No sub-goal for RUN-ALL goal '" ?id "'")))
)

(defrule run-all-goal-commit-to-subgoals-root
     (declare (salience ?*SALIENCE-HIGH*))
	?gf <- (goal (id ?root) (type ACHIEVE) (parent nil) (sub-type RUN-ALL-OF-SUBGOALS)
	             (committed-to $?committed) (mode EXPANDED))
	(goal (id ?sub-goal&:(not (member$ ?sub-goal ?committed))) (mode EXPANDED)
           (parent ?root) (type ACHIEVE) (priority ?priority))
	(not (goal (id ~?sub-goal) (parent ?root) (type ACHIEVE) (mode EXPANDED)
	           (priority ?priority2&:(> ?priority2 ?priority))))
	=>
	(modify ?gf (committed-to (create$ ?committed ?sub-goal)))
)

(defrule run-all-goal-commit-to-subgoals-intermediate
     (declare (salience ?*SALIENCE-HIGH*))
     (goal (id ?parent) (mode COMMITTED|DISPATCHED) (committed-to $? ?intermediate $?))
	?gf <- (goal (id ?intermediate) (type ACHIEVE) (parent ?parent)
                  (sub-type RUN-ALL-OF-SUBGOALS) (committed-to $?committed)
                  (mode EXPANDED))
	(goal (id ?sub-goal&:(not (member$ ?sub-goal ?committed))) (mode EXPANDED)
           (parent ?intermediate) (type ACHIEVE) (priority ?priority))
	(not (goal (id ~?sub-goal) (parent ?intermediate) (type ACHIEVE) (mode EXPANDED)
	           (priority ?priority2&:(> ?priority2 ?priority))))
	=>
	(modify ?gf (committed-to (create$ ?committed ?sub-goal)))
)


(defrule run-all-goal-commit
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS)
                  (committed-to $?committed) (mode EXPANDED))
	(forall (goal (id ?sub-goal&:(member$ ?sub-goal ?committed)) (parent ?id)
		         (type ACHIEVE) (priority ?priority))
	         (not (goal (id ?sub-goal2&:(not (member$ ?sub-goal2 ?committed)))
	            (parent ?id) (type ACHIEVE)
	            (priority ?priority)))
	 )
     (test (not (eq ?committed (create$))))
     =>
	(modify ?gf (mode COMMITTED))
)

(defrule run-all-goal-dispatch
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS) (mode COMMITTED)
	             (committed-to $? ?sub-goal $? )
	             (required-resources $?req)
	             (acquired-resources $?acq&:(subsetp ?req ?acq)))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode DISPATCHED))
	=>
	(modify ?gf (mode DISPATCHED))
)

(defrule run-all-goal-subgoal-evaluated
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to $? ?sub-goa $?))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode EVALUATED))
	=>
	(modify ?sg (mode RETRACTED))
)

(defrule run-all-goal-subgoal-rejected-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS)
                  (committed-to $? ?sb-goals $?))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE)(mode RETRACTED) (outcome REJECTED))
	=>
	(modify ?gf (mode FINISHED) (outcome REJECTED) (committed-to (create$ ))
	            (error SUB-GOAL-REJECTED)
	            (message (str-cat "Sub-goal '" ?sub-goal "' of RUN-ALL goal '" ?id
				      "' was rejected")))
)


(defrule run-all-goal-subgoal-failed-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to $? ?sub-goal $?))
	?sg <- (goal (id ?sub-goal) (type ACHIEVE) (parent ?id) (acquired-resources)
	             (mode EVALUATED) (outcome FAILED))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED) (committed-to (create$ ))
					(error SUB-GOAL-FAILED ?sub-goal)
					(message (str-cat "Sub-goal '" ?sub-goal "' of RUN-ALL goal '" ?id "' has failed")))
)

(defrule run-all-goal-subgoal-completed-one-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS) (mode DISPATCHED)
                  (committed-to $?committed))
	(forall (goal (id ?sub-goal&:(member$ ?sub-goal ?committed)) (type ACHIEVE) (parent ?id))
             (goal (id ?sub-goal) (acquired-resources) (mode RETRACTED) (outcome COMPLETED))
             )
	(goal (parent ?id) (type ACHIEVE) (mode EXPANDED))
	=>
	(modify ?gf (mode EXPANDED) (committed-to (create$)))
)

(defrule run-all-goal-subgoal-completed-all-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to $?committed))
	(forall (goal (id ?sub-goal&:(member$ ?sub-goal ?committed)) (type ACHIEVE) (parent ?id))
             (goal (id ?sub-goal) (acquired-resources) (mode RETRACTED) (outcome COMPLETED))
             )
	;?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	;             (type ACHIEVE) (mode RETRACTED) (outcome COMPLETED))
	(not (goal (parent ?id) (type ACHIEVE) (mode EXPANDED)))
	=>
	(modify ?gf (mode FINISHED) (outcome COMPLETED) (committed-to (create$ )))
)
