
;---------------------------------------------------------------------------
;  lock-actions.clp - CLIPS executive - lock action executors
;
;  Created: Tue Apr 24 21:32:19 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule lock-action
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status PENDING)
                      (action-name lock) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
	=>
	(bind ?lock-name (plan-action-arg name ?param-names ?param-values))
	(bind ?rv (robmem-mutex-try-lock (str-cat ?lock-name)))
	(modify ?pa (status (if ?rv then EXECUTION-SUCCEEDED else EXECUTION-FAILED)))
)

(defrule unlock-action
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status PENDING)
                      (action-name unlock) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
	=>
	(bind ?lock-name (plan-action-arg name ?param-names ?param-values))
	(bind ?rv (robmem-mutex-unlock (str-cat ?lock-name)))
	(modify ?pa (status (if ?rv then EXECUTION-SUCCEEDED else EXECUTION-FAILED)))
)
