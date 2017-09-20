
;---------------------------------------------------------------------------
;  skills.clp - CLIPS skill utilities
;
;  Created: Thu Dec 20 12:06:02 2012 (Train from Freiburg to Aachen)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate skiller-control
	(slot acquired (type SYMBOL) (allowed-values FALSE TRUE))
	(slot acquiring (type SYMBOL) (allowed-values FALSE TRUE))
  (multislot last-try (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
)

(defrule skill-init
	(ff-feature-loaded blackboard)
	=>
	(blackboard-open-reading "SkillerInterface" "Skiller")
	(assert (skiller-control))
)

(defrule skill-init-interface-opened
	(ff-feature-loaded blackboard)
	(blackboard-interface (type "SkillerInterface") (id "Skiller"))
	=>
	(path-load "skills.clp")
)

