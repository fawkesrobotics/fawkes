
;---------------------------------------------------------------------------
;  stalled-action.clp - CLIPS executive - detect and abort stalled actions
;
;  Created: Wed Feb 06 11:55:58 2019 +0100
;  Copyright  2012-2019  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule xm-stalled-action-pending
  "Detect if no action executor has been configured for an action.
   This builds on the requirement that an executor must pickup any
   relevant PENDING action immediately and at the very least mark
   it as WAITING. If this doesn't happen immediately, this rule
   will fire and fail the action."
  (declare (salience ?*SALIENCE-LOW*))
  (goal (id ?goal-id) (mode DISPATCHED) (committed-to ?plan-id))
  (plan (id ?plan-id) (goal-id ?goal-id))
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (action-name ?action-name) (state PENDING))
 =>
  (modify ?pa (state FAILED)
              (error-msg (str-cat "No action executor for action '" ?action-name "'")))
)
