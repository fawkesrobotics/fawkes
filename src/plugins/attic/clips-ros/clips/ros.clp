
;---------------------------------------------------------------------------
;  ros - ROS integration for CLIPS
;
;  Created: Tue Oct 22 18:21:39 2013
;  Copyright  2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate ros-node
  (slot name (type STRING))
  (multislot published)
  (multislot subscribed (type STRING))
  (multislot services (type STRING))
)

(deftemplate ros-topic
  (slot name (type STRING))
  (slot type (type STRING))
)

(deftemplate ros-topic-connection
  (slot topic (type STRING))
  (slot from (type STRING))
  (slot to (type STRING))
)

; Unused, simply will not assert ros-node in that case
;(deftemplate ros-node-unreachable
;  (slot name (type STRING))
;)

(deffunction ros-cleanup ()
  (delayed-do-for-all-facts ((?rn ros-node)) TRUE (retract ?rn))
  (delayed-do-for-all-facts ((?rt ros-topic)) TRUE (retract ?rt))
  (delayed-do-for-all-facts ((?rt ros-topic-connection)) TRUE (retract ?rt))
  ;(delayed-do-for-all-facts ((?rn ros-node-unreachable)) TRUE (retract ?rn))
)
