
/***************************************************************************
 *  navigator_thread.h - Robotino ROS Navigator Thread
 *
 *  Created: Sat June 09 15:13:27 2012
 *  Copyright  2012  Sebastian Reuter
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
 
#ifndef __ROS_NAVIGATOR_THREAD_H_
#define __ROS_NAVIGATOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/tf.h>
#include <aspect/clock.h>
#include <plugins/ros/aspect/ros.h>

#include <interfaces/NavigatorInterface.h>

#include <tf/types.h>
#include <math.h>
#include <ros/ros.h>
#include <utils/math/angle.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>

namespace fawkes {
  class NavigatorInterface;
}

class RosNavigatorThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ROSAspect,
  public fawkes::TransformAspect
{
 public:
  RosNavigatorThread(std::string &cfg_prefix);

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void check_status();
  void send_goal();
  void stop_goals();
  void load_config();
  bool set_dynreconf_value(const std::string& path, const float value);

 private:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  void activeCb();
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
  void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResultConstPtr& result);

  void transform_to_fixed_frame();

  fawkes::NavigatorInterface *nav_if_;
  MoveBaseClient *ac_;
  move_base_msgs::MoveBaseGoal goal_;
  bool cmd_sent_;
  bool connected_history_;

  fawkes::Time *ac_init_checktime_;
  
  std::string cfg_prefix_;

  // ROS dynamic reconfigure parts
  dynamic_reconfigure::ReconfigureRequest dynreconf_srv_req;
  dynamic_reconfigure::ReconfigureResponse dynreconf_srv_resp;
  dynamic_reconfigure::DoubleParameter dynreconf_double_param;
  dynamic_reconfigure::Config dynreconf_conf;

  std::string cfg_dynreconf_path_;
  std::string cfg_dynreconf_x_vel_name_;
  std::string cfg_dynreconf_y_vel_name_;
  std::string cfg_dynreconf_rot_vel_name_;

  std::string cfg_fixed_frame_;
  float cfg_ori_tolerance_;
  float cfg_trans_tolerance_;

  float param_max_vel;
  float param_max_rot;
  
  geometry_msgs::PoseStamped base_position;
  float goal_position_x;
  float goal_position_y;
  float goal_position_yaw;

};

#endif /* __ROS_NAVIGATOR_THREAD_H_ */
