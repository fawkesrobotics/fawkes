;---------------------------------------------------------------------------
;  simple.clp - CLIPS executive - single goal without parent or children
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
; Sub-type: SIMPLE
; Perform: itself
;
; A SIMPLE goal has no children. If the goal has no parent it gets selected
; automatically. The user expands and evaluates the goal. Cleanup is automated
; as well. Thefore SIMPLE goals are suited to act as leaf nodes of trees or
; goals without any goal structure that should be executed directly after
; formulation.
;
; Interactions:
; - User FORMULATES goal
; - Automatic:  SELECT goal if it has no parent, else User has to SELECT it
; - User EXPANDS goal
; - Automatic: COMMIT and DISPATCH goal
; - User EVALUATES goal
; - Automatic: Clean up any attached goals and RETRACT goal


(defrule simple-goal-select
  ?g <- (goal (parent nil) (id ?goal-id) (sub-type SIMPLE) (mode FORMULATED))
  (not (goal (parent ?goal-id)))
=>
  (modify ?g (mode SELECTED))
)


(defrule simple-goal-commit
  ?g <- (goal (id ?goal-id) (sub-type SIMPLE) (mode EXPANDED))
  (not (goal (parent ?goal-id)))
=>
  (modify ?g (mode COMMITTED))
)


(defrule simple-goal-fail-because-of-subgoal
  ?g <- (goal (id ?goal-id) (sub-type SIMPLE)
              (mode ~FINISHED&~EVALUATED&~RETRACTED))
  (goal (parent ?goal-id))
=>
  (modify ?g (mode FINISHED) (outcome FAILED)
              (error SUB-GOAL)
              (message (str-cat "Sub-goal for SIMPLE goal '" ?goal-id "'")))
)


(defrule simple-goal-dispatch
  ?g <- (goal (id ?goal-id) (type ACHIEVE) (sub-type SIMPLE) (mode COMMITTED)
              (class ?type)  (committed-to nil) (required-resources $?req)
							(verbosity ?verbosity)
              (acquired-resources $?acq&:(subsetp ?req ?acq)))
  (not (goal (parent ?goal-id)))
=>
  (if (neq ?verbosity QUIET) then
    (printout t "Goal " ?goal-id " DISPATCHED!" crlf)
  )
  (modify ?g (mode DISPATCHED))
)


(defrule simple-goal-retract
 ?g <- (goal (id ?goal-id) (type ACHIEVE) (sub-type SIMPLE) (mode EVALUATED)
             (acquired-resources))
  (not (goal (parent ?goal-id)))
=>
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
    (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
      (do-for-all-facts ((?g pddl-grounding)) (eq ?a:precondition ?g:id)
        (retract ?g)
      )
      (retract ?a)
    )
    (retract ?p)
  )
 (modify ?g (mode RETRACTED))
)
