
;---------------------------------------------------------------------------
;  plan.clp - CLIPS executive - plan representation
;
;  Created: Wed Sep 20 15:47:23 2017
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; (deftemplate durative-action
; 	(slot name (type SYMBOL))
; 	(slot duration (type FLOAT))
; 	(multislot params)
; )

; (deftemplate action-precondition
; 	(slot action-name (type SYMBOL))
; 	(slot type (type SYMBOL) (allowed-values POSITIVE NEGATIVE) (default POSITIVE))
; 	(slot predicate (type SYMBOL))
; 	(multislot values)
; )

(defmodule PLAN (import MAIN ?ALL) (import EXECUTIVE-PRIORITIES ?ALL) (export ?ALL))

(deftemplate PLAN::plan
	(slot name (type SYMBOL))
	(slot cost (type FLOAT))
)

(deftemplate PLAN::plan-action
	(slot id (type INTEGER))
	(slot plan-name (type SYMBOL))
	(slot action-name (type SYMBOL))
	(multislot params)
	(slot duration (type FLOAT))
	(slot dispatch-time (type FLOAT))
	(slot status (type SYMBOL) (allowed-values PENDING WAITING RUNNING FINAL FAILED))
)

; alternative
; (deftemplate PLAN::plan-action-parameter
; 	(slot plan-action-id (type INTEGER))
; 	(slot key)
; 	(slot value)
; )

(defrule PLAN::init
	(declare (salience ?*PRIORITY-INIT*) (auto-focus TRUE))
	(executive-init)
	=>
	(assert (module-initialized PLAN-EXEC))
)
