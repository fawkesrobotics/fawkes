
;---------------------------------------------------------------------------
;  temporal-missed-dispatch.clp -
;  Check for missed dispatch times in temporal plan execution
;
;  Created: Thu Feb 07 00:23:57 2019 +0100
;  Copyright  2012-2019  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule xm-temporal-plan-missed-dispatch
  "Detect when actions miss their intended dispatch time.
   For temporal plans, actions have intended dispatch times. If they cannot
   be met, for example because preconditions of the causal model are not
   satisfied, raise a warning indicating the specific action ID and op."
  (declare (salience ?*SALIENCE-LOW*))
  (time $?now)

  (plan (id ?plan-id) (goal-id ?goal-id) (type TEMPORAL)
        (start-time $?st&:(neq ?st (create$ 0 0))))

  ; An action which is due for execution hasn't been started yet
  (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (action-name ?op)
               (state FORMULATED) (dispatch-time ?dt&:(timeout ?now ?st ?dt)))

  ?g <- (goal (id ?goal-id) (mode DISPATCHED) (committed-to ?plan-id)
              (warning $?w&~:(member$ (create$ ACTION-MISSED-DISPATCH ?id ?op) ?w)))
=>
  (printout warn
            "Action " ?op " (goal " ?goal-id ", plan " ?plan-id ", action " ?id ")"
            " missed its dispatch time" crlf)

  (modify ?g (warning ?w ACTION-MISSED-DISPATCH ?id ?op))
)
