
;---------------------------------------------------------------------------
;  navgraph.clp - CLIPS navgraph access
;
;  Created: Sat Jun 22 02:21:44 2013
;  Copyright  2012-2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate navgraph
  (slot name (type STRING))
  (slot root (type STRING))
)

(deftemplate navgraph-node
  (slot name (type STRING))
  (slot x (type FLOAT))
  (slot y (type FLOAT))
  (multislot properties (type STRING))
)

(deftemplate navgraph-edge
  (slot from (type STRING))
  (slot to (type STRING))
  (slot directed (type SYMBOL) (allowed-values FALSE TRUE))
  (multislot properties (type STRING))
)
