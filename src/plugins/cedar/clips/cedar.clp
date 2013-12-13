
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

(defrule cedar-ros-enable
  (cedar-init)
  (ff-feature ros)
  (not (ff-feature-loaded ros))
  (confval (path "/cedar/use-ros") (type BOOL) (value TRUE))
  =>
  (printout warn "Enabling ROS monitoring" crlf)
  (ff-feature-request "ros")
  (path-load "cedar-ros.clp")
  (path-load "ros-model.clp")
)

(defrule cedar-blackboard-enable
  (cedar-init)
  (ff-feature blackboard)
  (confval (path "/cedar/use-fawkes") (type BOOL) (value TRUE))
  =>
  (printout warn "Enabling blackboard monitoring" crlf)
  (ff-feature-request "blackboard")
  (path-load "cedar-fawkes.clp")
)

(defrule cedar-start
  (declare (salience -10000))
  (cedar-init)
  =>
  (reset)
  (assert (cedar-start))
)
