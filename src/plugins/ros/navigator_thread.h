
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

#include <interfaces/NavigatorInterface.h>

#include <tf/types.h>
#include <math.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>

namespace fawkes {
  class NavigatorInterface;
}

class RosNavigatorThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect
{
 public:
  RosNavigatorThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void check_status();
  void send_goal();
  void stop_goals();

 private:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  fawkes::NavigatorInterface *nav_if_;
  MoveBaseClient *ac_;
  move_base_msgs::MoveBaseGoal goal_;
  bool cmd_sent_;
  bool connected_history_;
};

#endif /* __ROS_NAVIGATOR_THREAD_H_ */
