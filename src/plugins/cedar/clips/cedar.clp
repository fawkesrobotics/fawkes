
;---------------------------------------------------------------------------
;  cedar.clp - CEDAR system monitoring
;
;  Created: Mon Oct 28 14:19:31 2013
;  Copyright  2011-2013  Tim Niemueller [www.niemueller.de]
;             2011-2013  RWTH Aachen University (KBSG)
;             2011       SRI International
;             2011       Carnegie Mellon University
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(defglobal
  ?*CONFIG_PREFIX* = "/cedar"
)

(path-load "syscond.clp")

(defrule cedar-init
  (declare (salience 10000))
  (cedar-init)
  =>
  (printout t "CEDAR starting" crlf)
  (watch facts)
  (watch rules)
  (config-load ?*CONFIG_PREFIX*)
)

(defrule cedar-start
  (declare (salience -10000))
  (cedar-init)
  =>
  (reset)
  (assert (cedar-start))
)
