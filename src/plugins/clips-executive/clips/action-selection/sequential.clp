
;---------------------------------------------------------------------------
;  sequential.clp - CLIPS executive - action selection for sequential plans
;
;  Created: Thu Sep 21 22:17:50 2017 +0200
;  Copyright  2012-2019  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule action-selection-sequential-select
  (goal (id ?goal-id) (mode DISPATCHED) (committed-to ?plan-id))
  (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (state FORMULATED) (executable TRUE))
  (not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state ~FORMULATED&~FINAL)))
  (not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FORMULATED) (id ?oid&:(< ?oid ?id))))
 =>
  (modify ?pa (state PENDING))
)

(defrule action-selection-sequential-done
  ?g <- (goal (id ?goal-id) (mode DISPATCHED) (committed-to ?plan-id))
  (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
  (not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state ~FINAL)))
 =>
  (modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-sequential-failed
  ?g <- (goal (id ?goal-id) (mode DISPATCHED) (committed-to ?plan-id))
  (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
  (plan-action (goal-id ?goal-id) (plan-id ?plan-id)
               (action-name ?action-name) (state FAILED)
               (error-msg ?msg))
 =>
  (modify ?g (mode FINISHED) (outcome FAILED)
          (error ACTION-FAILED)
          (message (str-cat "Action " ?action-name " failed: " ?msg)))
)
