
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
  (slot transisiton (type STRING))
  (slot executable (type SYMBOL) (allowed-values FALSE TRUE))
)

