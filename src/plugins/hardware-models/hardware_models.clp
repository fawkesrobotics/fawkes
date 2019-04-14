
;---------------------------------------------------------------------------
;  hardware-models.clp - CLIPS hardware models helper functions
;
;  Created: Mon Mar 25 17:24:53 2019
;  Copyright  2019  Daniel Habering
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate hm-component
  (slot name (type SYMBOL))
  (slot state (type SYMBOL))
)

(deftemplate hm-edge
  (slot component (type SYMBOL))
  (slot from (type SYMBOL))
  (slot to (type SYMBOL))
  (slot transition (type SYMBOL))
  (slot probability (type FLOAT) (default 0.0))
  (slot executable (type SYMBOL) (allowed-values FALSE TRUE))
)

(deftemplate hm-transition
  (slot component (type SYMBOL))
  (slot transition (type SYMBOL))
)

