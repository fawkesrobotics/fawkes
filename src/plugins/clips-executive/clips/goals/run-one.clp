;---------------------------------------------------------------------------
;  run-one.clp - CLIPS executive - goal to run sub-goals
;
;  Created: Fri Jun 01 16:48:15 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Sub-type: RUN-ONE-OF-SUBGOALS
; Perform: one goal at a time, ordered by goal priority
; Succeed: if exactly one sub-goal succeeds
; Fail:    if exactly one sub-goal fails
; Reject:  if all sub-goals are rejected
;
; A RunOne parent goal will order the goals by priority and then start
; performing them in order. For goals which are rejected, the goal
; continues with the next goal. If any goal succeeds or fails, the
; parent succeeds or fails respectively.
;
; Interactions:
; - User FORMULATES goal
; - User SELECTS goal
; - User EXPANDS goal, consisting of:
;   * create goals with parent ID equal the RUN-ONE goal ID
;   * set RUN-ONE goal mode to EXPANDED
; - Automatic: if no sub-goal formulated -> FAIL
; - Automatic: if all sub-goals rejected -> REJECT
; - Automatic: take highest FORMULATED sub-goal and COMMIT to
; - Automatic: DISPATCH committed sub-goal by SELECTING it
; - User: handle sub-goal expansion, commiting, dispatching
; - Automatic: when sub-goal is EVALUATED, outcome determines parent goal:
;   * REJECTED: mode EXPANDED (re-try with other sub-goal)
;   * FAILED: mode FINISHED, outcome FAILED, message
;   * COMPLETED: mode FINISHED, outcome COMPLETED
;   -> Sub-goal is RETRACTED.
; User: EVALUATE goal
; User: RETRACT goal

(defrule run-one-goal-expand-failed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ONE-OF-SUBGOALS) (mode EXPANDED))
	(not (goal (type ACHIEVE) (parent ?id)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
					(message (str-cat "No sub-goal for RUN-ONE goal '" ?id "'")))
)

(defrule run-one-goal-commit
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ONE-OF-SUBGOALS) (mode EXPANDED))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED)
	      (priority ?priority))
	(not (goal (id ~?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED)
	           (priority ?priority2&:(> ?priority2 ?priority))))
	=>
	(modify ?gf (mode COMMITTED) (committed-to ?sub-goal))
)

(defrule run-one-goal-reject
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ONE-OF-SUBGOALS) (mode EXPANDED))
	(forall (goal (id ?sub-goal) (parent ?id) (type ACHIEVE))
		(goal (id ?sub-goal) (mode RETRACTED) (outcome REJECTED)))
	=>
	(modify ?gf (mode FINISHED) (outcome REJECTED) (committed-to nil)
	        (error SUB-GOALS-REJECTED))
)

(defrule run-one-goal-dispatch
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ONE-OF-SUBGOALS) (mode COMMITTED)
	             (committed-to ?sub-goal)
	             (required-resources $?req)
	             (acquired-resources $?acq&:(subsetp ?req ?acq)))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(modify ?gf (mode DISPATCHED))
	(modify ?sg (mode SELECTED))
)

(defrule run-one-goal-subgoal-evaluated
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ONE-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode EVALUATED))
	=>
	(modify ?sg (mode RETRACTED))
)

(defrule run-one-goal-subgoal-rejected-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ONE-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome REJECTED))
	=>
	(modify ?gf (mode EXPANDED) (committed-to nil))
)

(defrule run-one-goal-subgoal-failed-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ONE-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome FAILED))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED) (committed-to nil)
					(error SUB-GOAL-FAILED)
					(message (str-cat "Sub-goal '" ?sub-goal "' of RUN-ONE goal '" ?id "' has failed")))
)

(defrule run-one-goal-subgoal-completed-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-ONE-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome COMPLETED))
	=>
	(modify ?gf (mode FINISHED) (outcome COMPLETED) (committed-to nil))
)
