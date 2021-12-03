;****************************************************************************
;  action-selection.pddl: Test selection for promises
;
;  Created: Thu Dec 2 2021
;  Copyright  2021 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;****************************************************************************

(defrule action-selection-select
	?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
                      (id ?id) (state FORMULATED)
                      (action-name ?action-name)
                      (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (class ?class) (mode DISPATCHED) (verbosity ?verbosity))

  (not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FORMULATED) (id ?oid&:(< ?oid ?id))))
	=>
  (if (neq ?verbosity QUIET) then
    (printout t "Selected next action " ?action-name ?param-values crlf)
  )
	(modify ?pa (state PENDING))
)

(defrule action-selection-done
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (class ~EXPLORATION) (mode DISPATCHED) (type ACHIEVE))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state ~FINAL)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (class ?class& : (neq ?class EXPLORATION)) (mode DISPATCHED))
	(plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FAILED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)
