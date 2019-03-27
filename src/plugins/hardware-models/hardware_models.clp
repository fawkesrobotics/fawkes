
;---------------------------------------------------------------------------
;  hardware-models.clp - CLIPS hardware models helper functions
;
;  Created: Mon Mar 25 17:24:53 2019
;  Copyright  2019  Daniel Habering
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate hm-component
  (slot name (type STRING))
  (slot state (type STRING))
)

(deftemplate hm-edge
  (slot component (type STRING))
  (slot from (type STRING))
  (slot to (type STRING))
  (slot transition (type STRING))
  (slot probability (type FLOAT) (default 0.0))
  (slot executable (type SYMBOL) (allowed-values FALSE TRUE))
)

(deftemplate hm-transition
  (slot component (type STRING))
  (slot transition (type STRING))
)

(defrule hm-execute-transition
  ?t <- (hm-transition (component ?comp) (transition ?trans))
  ?c <- (hm-component (name ?comp) (state ?state))
  (hm-edge (component ?comp) (from ?state) (to ?to) (transition ?trans))
  =>
  (modify ?c (state ?to))
  (retract ?t)
)

(defrule hm-invalid-transition
  ?t <- (hm-transition (component ?comp) (transition ?trans))
  (hm-component (name ?comp) (state ?state))
  (not (hm-edge (component ?comp) (from ?state) (transition ?trans)))
  =>
  (retract ?t)
  (printout error "Invalid transition " ?trans " for component " ?comp " in state " ?state crlf)
)
