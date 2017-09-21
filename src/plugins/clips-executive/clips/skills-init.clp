
;---------------------------------------------------------------------------
;  skills.clp - CLIPS skill utilities
;
;  Created: Thu Dec 20 12:06:02 2012 (Train from Freiburg to Aachen)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defmodule SKILL-EXEC
	(import MAIN ?ALL)
	(import EXECUTIVE-PRIORITIES ?ALL)
	(export deftemplate ?ALL)
	(export deffunction skill-call)
)

(deftemplate SKILL-EXEC::skiller-control
	(slot acquired (type SYMBOL) (allowed-values FALSE TRUE))
	(slot acquiring (type SYMBOL) (allowed-values FALSE TRUE))
  (multislot last-try (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
)

(defrule SKILL-EXEC::init
	(declare (salience ?*PRIORITY-INIT*) (auto-focus TRUE))
	(ff-feature-loaded blackboard)
	=>
	(printout t "SKILL-EXEC::init" crlf)
	(blackboard-open-reading "SkillerInterface" "Skiller")
	(assert (skiller-control))
	(path-load "skills.clp")
	(assert (module-initialized SKILL-EXEC))
	(printout t "SKILL-EXEC::init done" crlf)
)
