
;---------------------------------------------------------------------------
;  temporal.clp - CLIPS executive - action selection for temporal plans
;
;  Created: Thu Sep 21 22:17:50 2017 +0200
;  Copyright  2012-2019  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule action-selection-temporal-zerotime
  (goal (id ?goal-id) (mode DISPATCHED) (committed-to ?plan-id))
  ?p <- (plan (id ?plan-id) (goal-id ?goal-id) (type TEMPORAL) (start-time 0 0))
 =>
  (modify ?p (start-time (now)))
)

(defrule action-selection-temporal-select
  (time $?now)
  (goal (id ?goal-id) (mode DISPATCHED) (committed-to ?plan-id))
  (plan (id ?plan-id) (goal-id ?goal-id) (type TEMPORAL) (start-time $?start-time))
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (state FORMULATED) (executable TRUE)
                      (dispatch-time ?dt&:(timeout ?now ?start-time ?dt)))
 =>
  (modify ?pa (state PENDING) (start-time ?now))
)

(defrule action-selection-temporal-done
  ?g <- (goal (id ?goal-id) (mode DISPATCHED) (committed-to ?plan-id))
  (plan (id ?plan-id) (goal-id ?goal-id) (type TEMPORAL))
  (not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state ~FINAL)))
 =>
  (modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-temporal-failed
  ?g <- (goal (id ?goal-id) (mode DISPATCHED) (committed-to ?plan-id))
  (plan (id ?plan-id) (goal-id ?goal-id) (type TEMPORAL))
  (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FAILED))
 =>
  (modify ?g (mode FINISHED) (outcome FAILED))
)
