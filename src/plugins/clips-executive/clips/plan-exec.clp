
;---------------------------------------------------------------------------
;  plan-exec.clp - CLIPS executive - plan execution
;
;  Created: Wed Sep 20 15:46:48 2017
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defmodule PLAN-EXEC
	(export ?ALL)
	(import EXECUTIVE-PRIORITIES ?ALL)
	(import MAIN ?ALL)
	(import PLAN ?ALL)
	(import SKILL-EXEC ?ALL)
)

(deftemplate PLAN-EXEC::action
	(slot action-id (type INTEGER))
)

(defrule PLAN-EXEC::init
	(declare (salience ?*PRIORITY-INIT*) (auto-focus TRUE))
	(executive-init)
	=>
	(assert (module-initialized PLAN-EXEC))
)

(defrule PLAN-EXEC::action-start
	?pa <- (plan-action (plan-name ?plan-name) (id ?id) (status PENDING)
											(action-name ?action-name) (params $?params))
	(not (plan-exec-action))
	(skiller-control (acquired TRUE))
	=>
	(skill-call ?action-name ?params)
	(modify ?pa (status WAITING))
	(assert (action (action-id ?id)))
)

(defrule PLAN-EXEC::action-running
	?pa <- (plan-action (plan-name ?plan-name) (id ?id) (status WAITING)
											(action-name ?action-name))
	(skill (name ?action-name) (status S_RUNNING))
	=>
	(printout t "Action " ?action-name " is running" crlf)
	(modify ?pa (status RUNNING))
)

(defrule PLAN-EXEC::action-final
	?pe <- (action (action-id ?id))
	?pa <- (plan-action (plan-name ?plan-name) (id ?id)
											(action-name ?action-name))
	?sf <- (skill (name ?action-name) (status S_FINAL))
	=>
	(printout t "Execution of " ?action-name " completed successfully" crlf)
	(modify ?pa (status FINAL))
	(retract ?sf ?pe)
)

(defrule PLAN-EXEC::action-failed
	?pe <- (action (action-id ?id))
	?pa <- (plan-action (plan-name ?plan-name) (id ?id)
											(action-name ?action-name))
	?sf <- (skill (name ?action-name) (status S_FAILED) (error-msg ?error))
	=>
	(printout warn "Execution of " ?action-name " FAILED (" ?error ")" crlf)
	(modify ?pa (status FAILED))
	(retract ?sf ?pe)
)
