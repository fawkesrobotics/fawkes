
;---------------------------------------------------------------------------
;  plan.clp - CLIPS executive - plan representation
;
;  Created: Wed Sep 20 15:47:23 2017
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; This is just a dummy until we have a full spec
(deftemplate goal
	(slot id (type SYMBOL))
  (slot type (type SYMBOL) (allowed-values ACHIEVE MAINTAIN) (default ACHIEVE))
	(slot mode (type SYMBOL) (allowed-values FORMULATED SELECTED EXPANDED
																					 COMMITTED DISPATCHED COMPLETED FAILED))
  (slot parent (type SYMBOL))
)

(deftemplate plan
	(slot id (type SYMBOL))
	(slot goal-id (type SYMBOL))
	(slot cost (type FLOAT))
)

(deftemplate plan-action
	(slot id (type INTEGER))
	(slot plan-id (type SYMBOL))
	(slot action-name (type SYMBOL))
	(multislot param-names)
	(multislot param-values)
	(slot duration (type FLOAT))
	(slot dispatch-time (type FLOAT))
	(slot status (type SYMBOL) (allowed-values FORMULATED PENDING WAITING RUNNING FINAL FAILED))
  (slot executable (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
)

; alternative
; (deftemplate plan-action-parameter
; 	(slot plan-action-id (type INTEGER))
; 	(slot key)
; 	(slot value)
; )
