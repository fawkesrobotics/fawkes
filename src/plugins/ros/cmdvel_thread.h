/***************************************************************************
 *  cmdvel_thread.h - Translate ROS Twist messages to Navgiator transrot
 *
 *  Created: Fri Jun 1 13:29:39 CEST 2012
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

#ifndef __PLUGINS_ROS_CMDVEL_THREAD_H_
#define __PLUGINS_ROS_CMDVEL_THREAD_H_

#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blocked_timing.h>
#include <aspect/blackboard.h>
#include <plugins/ros/aspect/ros.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Twist.h>


namespace fawkes {
  class MotorInterface;
}

class ROSCmdVelThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ROSAspect
{
 public:
  ROSCmdVelThread();

  virtual void init();
  virtual void loop();
  virtual bool prepare_finalize_user();
  virtual void finalize();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

private:
  void stop(); //stops all Motors
  void send_transrot(float vx, float vy, float omega); //sends Controls to the Motors
  void twist_msg_cb(const geometry_msgs::Twist::ConstPtr &msg);

 private:
  fawkes::MotorInterface *motor_if_;
  ros::Subscriber sub_;
};

#endif
