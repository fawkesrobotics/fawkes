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
; - User FORMULATES goal
; - User SELECTS goal
; - User EXPANDS goal, consisting of:
;   * create goals with parent ID equal the RUN-ALL goal ID
;   * set RUN-ALL goal mode to EXPANDED
; - Automatic: if no sub-goal formulated -> FAIL
; - Automatic: if all sub-goals rejected -> REJECT
; - Automatic: take highest FORMULATED sub-goal and COMMIT to
; - Automatic: DISPATCH committed sub-goal by SELECTING it
; - User: handle sub-goal expansion, commiting, dispatching, evaluating
; - Automatic: when sub-goal is EVALUATED, outcome determines parent goal:
;   * REJECTED: mode FINISHED, outcome REJECTED, message
;   * FAILED: mode FINISHED, outcome FAILED, message
;   * COMPLETED: mode FINISHED, outcome COMPLETED
;   -> Sub-goal is RETRACTED.
; User: EVALUATE goal
; User: RETRACT goal


(defrule run-all-goal-expand-failed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS)
							 (mode EXPANDED))
	(not (goal (type ACHIEVE) (parent ?id)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
					(error NO-SUB-GOALS)
					(message (str-cat "No sub-goal for RUN-ALL goal '" ?id "'")))
)

(defrule run-all-goal-commit
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS) (mode EXPANDED))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED)
	      (priority ?priority))
	(not (goal (id ~?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED)
	           (priority ?priority2&:(> ?priority2 ?priority))))
	=>
	(modify ?gf (mode COMMITTED) (committed-to ?sub-goal))
)

(defrule run-all-goal-dispatch
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS) (mode COMMITTED)
	             (committed-to ?sub-goal)
	             (required-resources $?req)
	             (acquired-resources $?acq&:(subsetp ?req ?acq)))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(modify ?gf (mode DISPATCHED))
	(modify ?sg (mode SELECTED))
)

(defrule run-all-goal-subgoal-evaluated
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode EVALUATED))
	=>
	(modify ?sg (mode RETRACTED))
)

(defrule run-all-goal-subgoal-rejected-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome REJECTED))
	=>
	(modify ?gf (mode FINISHED) (outcome REJECTED) (committed-to nil)
	            (error SUB-GOAL-REJECTED)
	            (message (str-cat "Sub-goal '" ?sub-goal "' of RUN-ALL goal '" ?id
				      "' was rejected")))
)


(defrule run-all-goal-subgoal-failed-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome FAILED))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED) (committed-to nil)
					(error SUB-GOAL-FAILED ?sub-goal)
					(message (str-cat "Sub-goal '" ?sub-goal "' of RUN-ALL goal '" ?id "' has failed")))
)

(defrule run-all-goal-subgoal-completed-one-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome COMPLETED))
	(goal (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(modify ?gf (mode EXPANDED) (committed-to nil))
)

(defrule run-all-goal-subgoal-completed-all-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ALL-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome COMPLETED))
	(not (goal (parent ?id) (type ACHIEVE) (mode FORMULATED)))
	=>
	(modify ?gf (mode FINISHED) (outcome COMPLETED) (committed-to nil))
)
