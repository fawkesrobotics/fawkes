;---------------------------------------------------------------------------
;  run-endless.clp - CLIPS executive - parent goal that always gets reformulated
;
;  Created: Tue 05 Jan 2019 15:48:31 CET
;  Copyright  2019  Tarik Viehmann <tarik.viehmann@rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.
;
; Sub-type: RUN-ENDLESS
; Params:  (params frequency T $?outcomes)
; with frequency T: integer in seconds
;      outcomes: (possibly multiple) outcomes in the format retract-on-?outcome
;                on which to retract the goal
;
; Perform: single goal
; Succeed: if sub-goal succeeds within given time
; Fail:    if sub-goal fails or all subgoals are rejected
;
; A RUN-ENDLESS goal has to be populated exactly one subgoal by the user.
; It executes that sub-goal and finishes then with the same outcome.
; Then, if at least T seconds have passed since it was last formulated, the
; goal is reformulated.
; Upon reformulation the user has to formulate a fresh sub-goal. RUN-ENDLESS
; goals save the time when they are reformulated in the meta with the
; last-formulated key.
;
; Interactions:
; - User FORMULATES goal
; - User creates sub-goals when FORMULATING the RUN-ENDLESS goal
; - User SELECTS goal
; - Automatic: if no sub-goal formulated -> FAIL
; - Automatic: take highest FORMULATED sub-goal and COMMIT to
; - Automatic: DISPATCH committed sub-goal by SELECTING it
; - User handles sub-goal expanding committing and dispatching
; - Automatic: when sub-goal is EVALUATED, goal either fails or succeeds
; - Automatic: when sub-goal is REJECTED, goal selects another one if possibe,
;              else fails
; - User EVALUATES goal and deletes sub-goal
; - Automatic: retract evaluated sub-goal
; - Automatic: re-FORMULATE goal once at least T seconds have passed since it
;              was formulated last


(defrule run-endless-goal-failed-no-subgoal
  ?gf <- (goal (id ?id) (type MAINTAIN) (sub-type RUN-ENDLESS) (mode EXPANDED))
  (not (goal (type ACHIEVE) (parent ?id)))
=>
 (modify ?gf (mode FINISHED) (outcome FAILED) (error NO-SUB-GOAL)
             (message (str-cat "No sub-goal for RUN-ENDLESS goal '" ?id "'")))
)


(defrule run-endless-goal-failed-many-subgoals
  ?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RETRY-SUBGOAL)
               (mode EXPANDED))
  (goal (id ?id1) (type ACHIEVE) (parent ?id))
  (goal (id ?id2&~?id1) (type ACHIEVE) (parent ?id))
=>
  (modify ?gf (mode FINISHED) (outcome FAILED) (error TOO-MANY-SUBGOALS)
              (message (str-cat "More than one sub-goal for RUN-ENDLESS goal '"
                                ?id "'")))
)


(defrule run-endless-goal-failed-invalid-params
  ?gf <- (goal (id ?id) (type MAINTAIN) (sub-type RUN-ENDLESS) (mode EXPANDED)
               (params $?params&~:(member$ frequency ?params)))
=>
  (modify ?gf (mode FINISHED) (outcome FAILED) (error INVALID-PARAMETERS)
              (message (str-cat "Invalid parameters for RUN-ENDLESS goal '"
                       ?id "'")))
)


(defrule run-endless-goal-commit
  ?g <- (goal (id ?id) (type MAINTAIN) (sub-type RUN-ENDLESS) (mode EXPANDED)
              (params $? frequency ? $?) (parent nil))
  (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED)
        (priority ?priority))
=>
  (modify ?g (mode COMMITTED) (committed-to ?sub-goal))
)


(defrule run-endless-goal-dispatch
  ?g <- (goal (mode COMMITTED) (id ?goal-id) (parent nil) (type MAINTAIN)
              (committed-to ?sub-goal) (sub-type RUN-ENDLESS)
              (required-resources $?req)
              (acquired-resources $?acq&:(subsetp ?req ?acq)))
  ?sg <- (goal (id ?sub-goal) (parent ?goal-id) (mode FORMULATED))
=>
  (modify ?g (mode DISPATCHED) (committed-to ?sub-goal))
  (modify ?sg (mode SELECTED))
)


(defrule run-endless-goal-finish
  ?pg <- (goal (id ?pg-id) (mode DISPATCHED) (type MAINTAIN)
               (sub-type RUN-ENDLESS))
  ?sg <- (goal (id ?sg-id) (parent ?pg-id) (mode RETRACTED)
               (acquired-resources) (outcome ?sg-out&:(neq ?sg-out REJECTED)))
=>
  (modify ?pg (mode FINISHED) (outcome ?sg-out))
)


(defrule run-endless-goal-fail-all-subgoals-rejected
  ?pg <- (goal (id ?pg-id) (type MAINTAIN) (sub-type RUN-ENDLESS)
               (mode EXPANDED|COMMITTED|DISPATCHED))
  (not (goal (parent ?pg-id) (outcome ~REJECTED)))
=>
  (modify ?pg (mode FINISHED) (outcome FAILED))
)


(defrule run-endless-goal-subgoal-evaluated
  ?gf <- (goal (id ?id) (type MAINTAIN) (sub-type RUN-ENDLESS)
               (mode DISPATCHED) (committed-to ?sub-goal))
  ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode EVALUATED))
=>
  (modify ?sg (mode RETRACTED))
)


(defrule run-endless-goal-reformulate
  (time $?now)
  ?g <- (goal (id ?goal-id) (parent nil) (type MAINTAIN) (sub-type RUN-ENDLESS)
          (mode EVALUATED) (outcome ?outcome)
          (params frequency ?freq
                  $?params&:(not (member$ (sym-cat retract-on- ?outcome) ?params)))
          (meta last-formulated $?last&:(timeout ?now ?last ?freq) host ?host))
  (not (goal (parent ?goal-id)))
=>
  (modify ?g (mode FORMULATED) (outcome UNKNOWN) (committed-to nil)
             (meta last-formulated ?now host ?host))
)

(defrule run-endless-goal-retract
  ?g <- (goal (id ?goal-id) (parent nil) (type MAINTAIN) (sub-type RUN-ENDLESS)
              (mode EVALUATED) (outcome ?outcome)
              (params $?params&:(member$ (sym-cat retract-on- ?outcome) ?params)))
  (not (goal (parent ?goal-id)))
  =>
  (modify ?g (mode RETRACTED))
)
