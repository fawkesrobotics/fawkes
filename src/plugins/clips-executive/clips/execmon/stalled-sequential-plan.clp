
;---------------------------------------------------------------------------
;  stalled-sequential-plan.clp -
;  detect stalled sequential strictly progressing plans
;
;  Created: Wed Feb 06 11:25:16 2019 +0100
;  Copyright  2012-2019  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule xm-stalled-sequential-plan-next-action-not-executable
  "This rule detects sequential plans which have stalled. It applies the
   strict assumption that a plan will always immediately advance, that is,
   once an action has finished, the next one is immediately started.
   This is in line with PDDL plan execution semantics, where actions fully
   describe the development of predicates. This assumption is violated,
   for example, if the execution should wait for preconditions to be
   fulfilled, e.g., in the presence of sensed predicates which may change
   due to exogenous events. In this case, a more lenient or specific
   XM mode may be desirable."
  (declare (salience ?*SALIENCE-LOW*))
  ?g <- (goal (id ?goal-id) (mode DISPATCHED) (committed-to ?plan-id))
  (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))

  ; We do have at least one action which is still to be executed
  (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
               (action-name ?action-name)
               (state FORMULATED) (executable FALSE))

  ; There is no other FORMULATED action which comes earlier
  (not (plan-action (goal-id ?goal-id) (plan-id ?plan-id)
                    (state FORMULATED) (id ?oid&:(< ?oid ?id))))

  ; there is no action which is currently ongoing
  (not (plan-action (goal-id ?goal-id) (plan-id ?plan-id)
                    (state ~FORMULATED&~FINAL&~FAILED)))
 =>
  (modify ?g (mode FINISHED) (outcome FAILED)
             (error STALLED-NONE-EXECUTABLE)
             (message (str-cat "Action " ?action-name
                               " (goal " ?goal-id ", plan " ?plan-id ", action " ?id ")"
                               " not executable, but next in sequence")))
)
