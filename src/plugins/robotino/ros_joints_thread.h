
/***************************************************************************
 *  ros_joints_thread.h - Publish Robotino joint info via ROS
 *
 *  Created: Fri Mar 30 10:55:31 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef __PLUGINS_ROBOTINO_ROS_JOINTS_THREAD_H_
#define __PLUGINS_ROBOTINO_ROS_JOINTS_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <plugins/ros/aspect/ros.h>

#include <ros/publisher.h>
#include <sensor_msgs/JointState.h>

namespace fawkes {
  class RobotinoSensorInterface;
}

class RobotinoRosJointsThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::LoggingAspect,
  public fawkes::ROSAspect
{
 public:
  RobotinoRosJointsThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // members
  fawkes::RobotinoSensorInterface *sens_if_;

  ros::Publisher pub_joints_;
  sensor_msgs::JointState joint_state_msg_;

};


#endif
