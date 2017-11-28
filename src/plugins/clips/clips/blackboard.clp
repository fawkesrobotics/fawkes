
;---------------------------------------------------------------------------
;  blackboard.clp - CLIPS blackboard feature code
;
;  Created: Thu Dec 12 19:53:20 2013
;  Copyright  2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate blackboard-interface
  (slot id (type STRING))
  (slot type (type STRING))
	(slot uid (type STRING))
  (slot hash (type STRING))
	(slot serial (type INTEGER))
	(slot writing (type SYMBOL) (allowed-values FALSE TRUE))
)

(deftemplate blackboard-interface-info
  (slot id (type STRING))
  (slot type (type STRING))
  (slot hash (type STRING))
  (slot has-writer (type SYMBOL) (allowed-values TRUE FALSE))
  (slot num-readers (type INTEGER))
  (slot writer  (type STRING))
  (multislot readers (type STRING))
  (multislot timestamp (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
)
