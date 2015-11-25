
/***************************************************************************
 *  move_base_thread.h - Emulate ROS move_base
 *
 *  Created: Wed May 07 13:48:59 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */
 
#ifndef __ROS_MOVE_BASE_THREAD_H_
#define __ROS_MOVE_BASE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/tf.h>
#include <plugins/ros/aspect/ros.h>

#include <interfaces/NavigatorInterface.h>

#include <tf/types.h>
#include <cmath>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/server/simple_action_server.h>

namespace fawkes {
  class NavigatorInterface;
}

class RosMoveBaseThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::TransformAspect,
  public fawkes::ROSAspect
{
 public:
  RosMoveBaseThread();

  /* thread */
  virtual void init();
  virtual void finalize();
  virtual void once();
  virtual void loop();

 private:
  typedef enum {
    NAVGRAPH,
    COLLI
  } ExecType;

  typedef actionlib::ActionServer<move_base_msgs::MoveBaseAction> MoveBaseServer;

  void action_goal_cb(MoveBaseServer::GoalHandle goal, ExecType ext);
  void action_cancel_cb(MoveBaseServer::GoalHandle goal);
  void message_cb(geometry_msgs::PoseStampedConstPtr goal_pose, ExecType ext);

  void stop();
  move_base_msgs::MoveBaseResult   create_result();
  move_base_msgs::MoveBaseFeedback create_feedback();


 private:
  fawkes::NavigatorInterface *nav_navgraph_if_;
  fawkes::NavigatorInterface *nav_colli_if_;
  fawkes::NavigatorInterface *nav_if_;

  MoveBaseServer  *as_colli_;
  MoveBaseServer  *as_navgraph_;
  ros::Subscriber  sub_colli_;
  ros::Subscriber  sub_navgraph_;

  MoveBaseServer::GoalHandle as_goal_;
  geometry_msgs::PoseStamped goal_pose_;

  std::string cfg_base_frame_;
  std::string cfg_fixed_frame_;

  bool         exec_as_;
  bool         exec_request_;
  bool         exec_running_;
  ExecType     exec_type_;
  unsigned int exec_msgid_;
};

#endif /* __ROS_NAVIGATOR_THREAD_H_ */
