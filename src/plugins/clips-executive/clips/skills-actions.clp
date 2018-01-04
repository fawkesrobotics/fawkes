
;---------------------------------------------------------------------------
;  skills-actions.clp - CLIPS executive - execute skill actions
;
;  Created: Wed Sep 20 15:46:48 2017
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate skill-action-execinfo
	(slot action-id (type INTEGER))
	;(slot channel (type INTEGER))
	(slot skill-name (type SYMBOL))
	(multislot skill-args)
)

(deftemplate skill-action-mapping
	(slot name (type SYMBOL) (default ?NONE))
	(slot map-string (type STRING))
)

(defrule skill-action-init
	(confval (path "/clips-executive/spec") (type STRING) (value ?spec))
	(confval (path ?p&:(eq (str-index (str-cat "/clips-executive/specs/" ?spec "/action-mapping/") ?p) 1))
					 (type STRING) (value ?s))
	=>
	(bind ?prefix (str-cat "/clips-executive/specs/" ?spec "/action-mapping/"))
	(bind ?name (sym-cat (sub-string (+ (str-length ?prefix) 1) (str-length ?p) ?p)))
	(assert (skill-action-mapping (name ?name) (map-string ?s)))
)

(defrule skill-action-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status PENDING)
                      (action-name ?action-name) (executable TRUE)
                      (param-names $?params)
                      (param-values $?param-values))
	(skill-action-mapping (name ?action-name))
	(not (skill-action-execinfo))
	(skiller-control (acquired TRUE))
	=>
	(skill-call ?action-name ?params ?param-values)
	(modify ?pa (status WAITING))
	(bind ?args (create$))
	(loop-for-count (?i (length$ ?params))
		(bind ?args (append$ ?args (nth$ ?i ?params) (nth$ ?i ?param-values)))
	)
	(assert (skill-action-execinfo (action-id ?id) (skill-name ?action-name) (skill-args ?args)))
)

(defrule skill-action-running
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status WAITING)
											(action-name ?action-name))
	(skill (name ?action-name) (status S_RUNNING))
	=>
	(printout t "Action " ?action-name " is running" crlf)
	(modify ?pa (status RUNNING))
)

(defrule skill-action-final
	?pe <- (skill-action-execinfo (action-id ?id))
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
											(action-name ?action-name))
	?sf <- (skill (name ?action-name) (status S_FINAL))
	=>
	(printout t "Execution of " ?action-name " completed successfully" crlf)
	(modify ?pa (status FINAL))
	(retract ?sf ?pe)
)

(defrule skill-action-failed
	?pe <- (skill-action-execinfo (action-id ?id))
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
											(action-name ?action-name))
	?sf <- (skill (name ?action-name) (status S_FAILED) (error-msg ?error))
	=>
	(printout warn "Execution of " ?action-name " FAILED (" ?error ")" crlf)
	(modify ?pa (status FAILED))
	(retract ?sf ?pe)
)
