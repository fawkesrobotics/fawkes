
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

(defrule cedar-init
  (declare (salience 8000))
  (cedar-init)
  =>
  (printout t "CEDAR starting" crlf)
  (watch facts)
  (watch rules)
  (path-load "syscond.clp")
  (config-load ?*CONFIG_PREFIX*)
)

(defrule cedar-enable-redefine-warning
  (declare (salience 8200))
  (cedar-init)
  (ff-feature redefine-warning)
  (not (ff-feature-loaded redefine-warning))
  =>
  (printout t "Enabling redefinition warning feature" crlf)
  (ff-feature-request "redefine-warning")
)

(defrule cedar-enable-debug
  (cedar-init)
  (confval (path "/cedar/clips-debug") (type BOOL) (value TRUE))
  =>
  (printout t "CLIPS debugging enabled, watching facts and rules" crlf)
  (watch facts)
  (watch rules)
)

(defrule cedar-silence-debug-facts
  (declare (salience -1000))
  (cedar-init)
  (confval (path "/cedar/clips-debug") (type BOOL) (value TRUE))
  (confval (path "/cedar/unwatch-facts") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following facts: " ?lv crlf)
  (foreach ?v ?lv (unwatch facts (sym-cat ?v)))
)

(defrule cedar-silence-debug-rules
  (declare (salience -1000))
  (cedar-init)
  (confval (path "/cedar/clips-debug") (type BOOL) (value TRUE))
  (confval (path "/cedar/unwatch-rules") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following rules: " ?lv crlf)
  (foreach ?v ?lv (unwatch rules (sym-cat ?v)))
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
