
;---------------------------------------------------------------------------
;  cx-identity.clp - CLIPS executive - identity access
;
;  Created: Wed Apr 25 10:15:58 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deffunction cx-identity-set (?identity)
	(if (any-factp ((?wf wm-fact)) (eq ?wf:id "/cx/identity"))
	then
		(do-for-fact ((?wf wm-fact)) (eq ?wf:id "/cx/identity")
			(modify ?wf (value ?identity) (type STRING))
		)
	else
		(assert (wm-fact (id "/cx/identity") (type STRING) (value ?identity)))
	)
)

(deffunction cx-identity ()
	(bind ?identity "UNKNOWN")
	(do-for-fact ((?wf wm-fact)) (eq ?wf:id "/cx/identity")
		(bind ?identity ?wf:value)
	)
	(return ?identity)
)
