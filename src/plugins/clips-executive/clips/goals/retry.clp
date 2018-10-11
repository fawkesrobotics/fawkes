;---------------------------------------------------------------------------
;  retry.clp - CLIPS executive - goal to retry a sub-goal a number of times
;
;  Created: Mon Jun 04 17:15:05 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Sub-type: RETRY-SUBGOAL
; Perform: single goal, retry configurable number times
; Params:  (params max-tries <N>) for N a positive integer
; Succeed: if sub-goal succeeds within <N> tries
; Fail:    if sub-goal fails in the <N>th try
; Reject:  if sub-goal is rejected in the <N>th try
;
; A RETRY parent goal takes one and only one sub-goal. Once the RETRY goal
; has been expanded, the sub-goal is executed. If it sub-goal fails or is
; rejected, the RETRY goal re-selects the sub-goal. In total, the sub-goal
; is tried for up to N times with N being a positive configurable integer
; passed as the only max-tries parameter. If the last try fails, the RETRY
; goals fails, if the goal is rejected on the last try, the RETRY goal is
; rejected.
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
;   * if sub-goal rejected and num-tries < max-tries -> EXPANDED
;   * if sub-goal fails and num-tries < max-tries -> EXPANDED
;   * if sub-goal rejected and num-tries == max-tries -> FINISHED|REJECTED
;   * if sub-goal fails and num-tries == max-tries -> FINISHED|FAILED
;   -> Sub-goal is RETRACTED.
; User: EVALUATE goal
; User: RETRACT goal

(defrule retry-goal-failed-no-subgoal
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RETRY-SUBGOAL) (mode EXPANDED))
	(not (goal (type ACHIEVE) (parent ?id)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
					(error NO-SUB-GOAL)
					(message (str-cat "No sub-goal for RETRY goal '" ?id "'")))
)

(defrule retry-goal-failed-invalid-params
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RETRY-SUBGOAL) (mode EXPANDED)
	             (params $?params&~:(member$ max-tries ?params)|~:(= (length$ ?params) 2)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
	            (error INVALID-PARAMETERS)
	            (message (str-cat "Invalid parameters for RETRY goal '" ?id "'")))
)

(defrule retry-goal-failed-many-subgoals
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RETRY-SUBGOAL)
							 (mode EXPANDED))
	(goal (id ?id1) (type ACHIEVE) (parent ?id))
	(goal (id ?id2&~?id1) (type ACHIEVE) (parent ?id))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
	            (error TOO-MANY-SUBGOALS)
	            (message (str-cat "More than one sub-goal for RETRY goal '" ?id "'")))
)

(defrule retry-goal-add-meta
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RETRY-SUBGOAL) (mode EXPANDED)
	             (meta $?meta&~:(member$ num-tries ?meta)))
	=>
	(modify ?gf (meta num-tries 0))
)

(defrule retry-goal-auto-expand
	"If a sub-goal has already been specified, automatically switch to EXPANDED."
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RETRY-SUBGOAL) (mode SELECTED))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(modify ?gf (mode EXPANDED))
)

(defrule retry-goal-commit
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RETRY-SUBGOAL) (mode EXPANDED)
	             (params max-tries ?max-tries)
	             (meta num-tries ?num-tries&:(< ?num-tries ?max-tries)))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(bind ?new-num-tries (+ ?num-tries 1))
	(if (> ?new-num-tries 1) then
		(printout warn "Committing to RETRY " ?id " -> " ?sub-goal
		               " (try " ?new-num-tries "/" ?max-tries ")" crlf)
	)
	(modify ?gf (mode COMMITTED) (committed-to ?sub-goal) (meta num-tries ?new-num-tries))
)

(defrule retry-goal-dispatch
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RETRY-SUBGOAL) (mode COMMITTED)
	             (committed-to ?sub-goal)
	             (required-resources $?req)
	             (acquired-resources $?acq&:(subsetp ?req ?acq)))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(modify ?gf (mode DISPATCHED))
	(modify ?sg (mode SELECTED))
)

(defrule retry-goal-subgoal-evaluated
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RETRY-SUBGOAL) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode EVALUATED))
	=>
	(modify ?sg (mode RETRACTED))
)

(defrule retry-goal-subgoal-failed-or-rejected-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RETRY-SUBGOAL) (mode DISPATCHED)
	             (committed-to ?sub-goal)
	             (params max-tries ?max-tries)
	             (meta num-tries ?num-tries))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome ?outcome&FAILED|REJECTED))
	=>
	(if (= ?num-tries ?max-tries)
	then
		(modify ?gf (mode FINISHED) (outcome ?outcome) (committed-to nil)
		            (error (sym-cat SUB-GOAL- ?outcome))
		            (message (str-cat "RETRY goal '" ?id "' sub-goal '" ?sub-goal
		                              "' " ?outcome " in try " ?num-tries "/" ?max-tries)))
	else
		(modify ?sg (mode FORMULATED) (outcome UNKNOWN))
		(modify ?gf (mode EXPANDED) (committed-to nil))
	)
)

(defrule retry-goal-subgoal-completed-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RETRY-SUBGOAL) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome COMPLETED))
	=>
	(modify ?gf (mode FINISHED) (outcome COMPLETED))
)
