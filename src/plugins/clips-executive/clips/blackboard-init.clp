
;---------------------------------------------------------------------------
;  blackboard-init.clp - Initialize blackboard access
;
;  Created: Wed Sep 20 15:16:05 2017 +0200
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

;load needed interfaces
(defrule blackboard-enable-time-read
  "Prepare blackboard usage"
  (declare (salience 1000))
  (ff-feature blackboard)
  =>
  (blackboard-enable-time-read)
  (unwatch rules blackboard-read)
)

(defrule blackboard-preload
  "Preload configured blackboard interface types."
  (declare (salience 1000))
  (ff-feature blackboard)
  (confval (path "/clips-executive/spec") (type STRING) (value ?spec))
  (confval (path ?p&:(eq ?p (str-cat "/clips-executive/specs/" ?spec "/blackboard-preload")))
					 (type STRING) (is-list TRUE) (list-value $?preload-types))
  =>
  (printout t "Pre-loading blackboard interface types: " ?preload-types crlf)
  (foreach ?t ?preload-types (blackboard-preload ?t))
)
