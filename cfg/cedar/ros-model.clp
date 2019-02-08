
;---------------------------------------------------------------------------
;  ros-model.clp - CEDAR example system model for ROS
;
;  Created: Fri Dec 06 13:37:54 2013
;  Copyright  2011-2013  Tim Niemueller [www.niemueller.de]
;             2011-2013  RWTH Aachen University (KBSG)
;             2011       SRI International
;             2011       Carnegie Mellon University
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(deffacts cedar-model-ros
  (model-ros-topic (name "/chatter") (type "std_msgs/String"))
  (model-ros-node  (name "/talkerpub") (published "/chatter" "/rosout") (subscribed)
		   (services) (flags))
  (model-ros-node  (name "/talkersub") (published "/rosout") (subscribed "/chatter")
		   (services) (flags))
  (model-ros-node  (name "/fawkes") (published "/rosout") (subscribed "/cmd_vel") (services) (flags))
  (model-ros-topic-connection (topic "/rosout") (from "/fawkes") (to "/rosout"))
  (model-ros-topic-connection (topic "/chatter") (from "/talkerpub") (to "/talkersub"))
  ;(model-ros-topic-connection (topic "/rosout") (from "/fawkes") (to "/rostopic_3041_1386770440241"))
  ;(model-ros-topic-connection (topic "/cmd_vel") (from "/somewhere") (to "/fawkes"))
)
