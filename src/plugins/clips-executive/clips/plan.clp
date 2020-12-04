
;---------------------------------------------------------------------------
;  plan.clp - CLIPS executive - plan representation
;
;  Created: Wed Sep 20 15:47:23 2017
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate plan
	(slot id (type SYMBOL))
	(slot goal-id (type SYMBOL))
	(slot cost (type FLOAT))
	(slot type (type SYMBOL))
	(multislot start-time (type INTEGER) (cardinality 2 2) (default 0 0))
)

; TODO: Rename slots
(deftemplate plan-action
	(slot id (type INTEGER))
	(slot goal-id (type SYMBOL))
	(slot plan-id (type SYMBOL))
	(slot skiller (type STRING) (default "Skiller"))
	(slot action-name (type SYMBOL))
	(multislot param-names)
	(multislot param-values)
	(slot duration (type FLOAT))
	(slot dispatch-time (type FLOAT) (default -1.0))
	(slot state (type SYMBOL)
	            (allowed-values FORMULATED PENDING WAITING RUNNING EXECUTION-SUCCEEDED
	                            SENSED-EFFECTS-WAIT SENSED-EFFECTS-HOLD EFFECTS-APPLIED
	                            FINAL EXECUTION-FAILED FAILED))
	(slot executable (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
	(slot error-msg (type STRING))
	(multislot start-time (type INTEGER) (cardinality 2 2) (default 0 0))
)

(deffunction plan-action-arg (?param-name ?param-names ?param-values $?default)
	(foreach ?p ?param-names
		(if (eq ?param-name ?p) then (return (nth$ ?p-index ?param-values)))
	)
	(if (> (length$ ?default) 0) then (return (nth$ 1 ?default)))
	(return FALSE)
)

(deffunction plan-retract-all-for-goal (?goal-id)
	"Retract all plans associated with the given goal."
	(delayed-do-for-all-facts ((?plan plan)) (eq ?plan:goal-id ?goal-id)
		(delayed-do-for-all-facts ((?pa plan-action))
			(and (eq ?pa:goal-id ?goal-id) (eq ?pa:plan-id ?plan:id))
			(retract ?pa)
		)
		(retract ?plan)
	)
)
