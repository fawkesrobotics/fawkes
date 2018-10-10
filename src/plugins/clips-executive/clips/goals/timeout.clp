;---------------------------------------------------------------------------
;  timeout.clp - CLIPS executive - parent goal with timeout
;
;  Created: Mon Jun 11 12:03:33 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Sub-type: TIMEOUT-SUBGOAL
; Perform: single goal, execute with timeout
; Params:  (params timeout T) for a duration T as float in seconds
; Succeed: if sub-goal succeeds within given time
; Fail:    if sub-goal fails or does not complete within given time
; Reject:  if sub-goal is rejected
;
; A TIMEOUT parent goal takes one and only one sub-goal. It executes the
; sub-goal and forwards its response. However, if the goal does not complete
; within the given timebound, it is aborted and reports failure. The time
; starts when the parent goal enters the DISPATCHED mode.
;
; Interactions:
; - User FORMULATES goal
; - User SELECTS goal
; Options:
;   1. User EXPANDS goal when SELECTED, consisting of:
;      * create the single sub-goal with parent ID equal the RETRY goal ID
;      * set RETRY goal mode to EXPANDED
;   2. User already creates sub-goal when FORMULATING the RETRY goal
;      * Automatic: switching mode from SELECTED to EXPANDED without action
;      (a sub-goal already exists when switching to SELECTED mode)
; - Automatic: if no sub-goal formulated -> FAIL
; - Automatic: if more than one sub-goal formulated -> FAIL
; - Automatic: COMMIT sub-goal
; - Automatic: DISPATCH committed sub-goal by SELECTING it
; - Automatic: one of the following outcomes for the EVALUATED sub-goal:
;   * if sub-goal succeeds: -> FINISHED|COMLETED
;   * if sub-goal rejected -> FINISHED|REJECTED
;   * if sub-goal fails -> FINISHED|FAILED
; - Automatic: if sub-goal does not complete within configured time
;   sub goal -> FINISHED|FAILED, error TIMEOUT
;   (User needs to evaluate, then EVALUATED handling as shown above,
;    sub-goals must support being interrupted by directly going to FINISHED|FAILED).
;   -> Sub-goal is RETRACTED.
; User: EVALUATE goal
; User: RETRACT goal

(defrule timeout-goal-failed-no-subgoal
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TIMEOUT-SUBGOAL) (mode EXPANDED))
	(not (goal (type ACHIEVE) (parent ?id)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
	        (error NO-SUB-GOAL)
	        (message (str-cat "No sub-goal for TIMEOUT goal '" ?id "'")))
)

(defrule timeout-goal-failed-invalid-params
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TIMEOUT-SUBGOAL) (mode EXPANDED)
	             (params $?params&~:(member$ timeout ?params)|~:(= (length$ ?params) 2)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED) (error INVALID-PARAMETERS)
	        (message (str-cat "Invalid parameters for TIMEOUT goal '" ?id "'")))
)

(defrule timeout-goal-failed-many-subgoals
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TIMEOUT-SUBGOAL) (mode EXPANDED))
	(goal (id ?id1) (type ACHIEVE) (parent ?id))
	(goal (id ?id2&~?id1) (type ACHIEVE) (parent ?id))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
	        (error TOO-MANY-SUBGOALS)
	        (message (str-cat "More than one sub-goal for TIMEOUT goal '" ?id "'")))
)

(defrule timeout-goal-auto-expand
	"If a sub-goal has already been specified, automatically switch to EXPANDED."
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TIMEOUT-SUBGOAL) (mode SELECTED))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(modify ?gf (mode EXPANDED))
)

(defrule timeout-goal-commit
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TIMEOUT-SUBGOAL) (mode EXPANDED))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	(not (goal (id ?sub-goal-2&~?sub-goal) (type ACHIEVE) (parent ?id)))
	=>
	(modify ?gf (mode COMMITTED) (committed-to ?sub-goal))
)

(defrule timeout-goal-dispatch
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TIMEOUT-SUBGOAL) (mode COMMITTED)
	             (committed-to ?sub-goal)
	             (required-resources $?req)
	             (acquired-resources $?acq&:(subsetp ?req ?acq)))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(modify ?gf (mode DISPATCHED) (meta start-time (now)))
	(modify ?sg (mode SELECTED))
)

(defrule timeout-goal-subgoal-timeout
	(time $?now)
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TIMEOUT-SUBGOAL) (mode DISPATCHED)
	             (committed-to ?sub-goal)
	             (params timeout ?timeout)
	             (meta start-time $?start-time&:(timeout ?now ?start-time ?timeout)))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode ?mode&:(neq ?mode FINISHED EVALUATED RETRACTED)))
	=>
	(modify ?sg (mode FINISHED) (outcome FAILED) (error TIMEOUT)
	        (message (str-cat "Parent " ?id " set FAILED due to timeout")))
)

(defrule timeout-goal-subgoal-evaluated
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TIMEOUT-SUBGOAL) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode EVALUATED))
	=>
	(modify ?sg (mode RETRACTED))
)

(defrule timeout-goal-subgoal-failed-or-rejected-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TIMEOUT-SUBGOAL) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources) (error $?error)
	             (type ACHIEVE) (mode RETRACTED) (outcome ?outcome&FAILED|REJECTED))
 =>
	(bind ?error-code (if (> (length$ ?error) 0) then (nth$ 1 ?error) else (sym-cat SUBGOAL- ?outcome)))
	(modify ?gf (mode FINISHED) (outcome ?outcome) (committed-to nil) (error ?error-code)
	        (message (str-cat "TIMEOUT goal '" ?id "' sub-goal '" ?sub-goal
	                          "' " ?outcome " (error " ?error-code ")")))
)

(defrule timeout-goal-subgoal-completed-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type TIMEOUT-SUBGOAL) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome COMPLETED))
	=>
	(modify ?gf (mode FINISHED) (outcome COMPLETED))
)
