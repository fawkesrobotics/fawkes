
;---------------------------------------------------------------------------
;  blackboard-init.clp - Initialize blackboard access
;
;  Created: Wed Sep 20 15:16:05 2017 +0200
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(blackboard-enable-time-read)
(unwatch rules blackboard-read)

(do-for-fact ((?cs confval) (?cp confval))
						 (and (eq ?cs:path "/clips-executive/spec") (eq ?cs:type STRING)
									(eq ?cp:path (str-cat "/clips-executive/specs/" ?cs:value "/blackboard-preload"))
									(eq ?cp:type STRING) (eq ?cp:is-list TRUE))
	(foreach ?t ?cp:list-value (blackboard-preload ?t))
)
