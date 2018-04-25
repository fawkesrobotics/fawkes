
;---------------------------------------------------------------------------
;  lock-actions.clp - CLIPS executive - lock action executors
;
;  Created: Tue Apr 24 21:32:19 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule lock-actions-lock-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status PENDING)
                      (action-name lock) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
	=>
	(bind ?lock-name (plan-action-arg name ?param-names ?param-values))
	; The following performs a synchronous/blocking call
	;(bind ?rv (robmem-mutex-try-lock (str-cat ?lock-name)))
	;(modify ?pa (status (if ?rv then EXECUTION-SUCCEEDED else EXECUTION-FAILED)))
	(mutex-try-lock-async ?lock-name)
	(modify ?pa (status RUNNING))
)

(defrule lock-actions-lock-acquired
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
                      (action-name lock) (status RUNNING)
                      (param-names $?param-names) (param-values $?param-values))
	?mf <- (mutex (name ?name&:(eq ?name (plan-action-arg name ?param-names ?param-values)))
								(request LOCK) (response ACQUIRED))
	=>
	(modify ?pa (status EXECUTION-SUCCEEDED))
	(modify ?mf (request NONE) (response NONE))
)

(defrule lock-actions-lock-rejected
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
                      (action-name lock) (status RUNNING)
                      (param-names $?param-names) (param-values $?param-values))
	?mf <- (mutex (name ?name&:(eq ?name (plan-action-arg name ?param-names ?param-values)))
								(request LOCK) (response REJECTED|ERROR) (error-msg ?error-msg))
	=>
	(modify ?pa (status EXECUTION-FAILED) (error-msg ?error-msg))
	(modify ?mf (request NONE) (response NONE) (error-msg ""))
)

(defrule lock-actions-unlock-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status PENDING)
                      (action-name unlock) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
	=>
	(bind ?lock-name (plan-action-arg name ?param-names ?param-values))
	; The following performs a synchronous/blocking call
	;(bind ?rv (robmem-mutex-unlock (str-cat ?lock-name)))
	;(modify ?pa (status (if ?rv then EXECUTION-SUCCEEDED else EXECUTION-FAILED)))
	(mutex-unlock-async ?lock-name)
	(modify ?pa (status RUNNING))
)

(defrule lock-actions-unlock-done
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
                      (action-name unlock) (status RUNNING)
                      (param-names $?param-names) (param-values $?param-values))
	?mf <- (mutex (name ?name&:(eq ?name (plan-action-arg name ?param-names ?param-values)))
								(request UNLOCK) (response UNLOCKED))
	=>
	(modify ?pa (status EXECUTION-SUCCEEDED))
	(modify ?mf (request NONE) (response NONE))
)
