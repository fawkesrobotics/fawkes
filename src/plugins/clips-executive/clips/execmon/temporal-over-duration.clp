
;---------------------------------------------------------------------------
;  temporal-over-duration.clp -
;  Check for actions which run over their expected duration
;
;  Created: Thu Feb 07 13:11:35 2019 +0100
;  Copyright  2012-2019  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule xm-temporal-plan-warn-action-over-duration
  "This rule adds a warning to a goal for each action of a plan, that the
   the goal committed to, when the execution of an action takes longer
   than anticipated."
  (declare (salience ?*SALIENCE-LOW*))
  (time $?now)

  ; An ongoing action has not finished in the expected duration
  (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (action-name ?op)
               (state ~FORMULATED&~FINAL&~FAILED)
               (start-time $?st&:(neq ?st (create$ 0 0)))
               (duration ?duration&:(timeout ?now ?st ?duration)))

  ; The associated goal, ensure we have not already warning about this action
  ?g <- (goal (id ?goal-id) (mode DISPATCHED) (committed-to ?plan-id)
              (warning $?w&~:(member$ (create$ ACTION-OVER-DURATION ?id ?op) ?w)))
  (plan (id ?plan-id) (goal-id ?goal-id) (type TEMPORAL))
 =>
  (printout warn
            "Action " ?op " (goal " ?goal-id ", plan " ?plan-id ", action " ?id ")"
            " running longer than expected (which was " ?duration " sec)" crlf)
  (modify ?g (warning ?w ACTION-OVER-DURATION ?id ?op))
)
