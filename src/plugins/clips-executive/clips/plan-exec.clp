

;---------------------------------------------------------------------------
;  plan-exec.clp - CLIPS executive - plan execution
;
;  Created: Wed Sep 20 15:46:48 2017
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate plan-exec-action
	(slot action-id (type INTEGER))
)

(defrule plan-exec-action-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status PENDING)
                      (action-name ?action-name) (param-names $?params)
                      (param-values $?param-values))
	(not (plan-exec-action))
	(skiller-control (acquired TRUE))
	=>
	(skill-call ?action-name ?params ?param-values)
	(modify ?pa (status WAITING))
	(assert (plan-exec-action (action-id ?id)))
)

(defrule plan-exec-action-running
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status WAITING)
											(action-name ?action-name))
	(skill (name ?action-name) (status S_RUNNING))
	=>
	(printout t "Action " ?action-name " is running" crlf)
	(modify ?pa (status RUNNING))
)

(defrule plan-exec-action-final
	?pe <- (plan-exec-action (action-id ?id))
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
											(action-name ?action-name))
	?sf <- (skill (name ?action-name) (status S_FINAL))
	=>
	(printout t "Execution of " ?action-name " completed successfully" crlf)
	(modify ?pa (status FINAL))
	(retract ?sf ?pe)
)

(defrule plan-exec-action-failed
	?pe <- (plan-exec-action (action-id ?id))
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
											(action-name ?action-name))
	?sf <- (skill (name ?action-name) (status S_FAILED) (error-msg ?error))
	=>
	(printout warn "Execution of " ?action-name " FAILED (" ?error ")" crlf)
	(modify ?pa (status FAILED))
	(retract ?sf ?pe)
)
