/***************************************************************************
 *  cmdvel_plugin.cpp - Translate ROS Twist messages to Navgiator transrot
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

#include "cmdvel_thread.h"
#include <interfaces/MotorInterface.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>

//using namespace ros;
using namespace fawkes;

/** @class ROSCmdVelThread "cmdvel_thread.h"
 * Thread to translate ROS twist messages to navigator transrot messages.
 * @author Sebastian Reuter
 */

/** Constructor. */
ROSCmdVelThread::ROSCmdVelThread()
  : Thread("ROSCmdVelThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

void
ROSCmdVelThread::init()
{
  std::string motor_if_id = config->get_string("/ros/cmdvel/motor_interface_id");
  motor_if_ = blackboard->open_for_reading<MotorInterface>(motor_if_id.c_str());
  sub_ = rosnode->subscribe("cmd_vel", 10, &ROSCmdVelThread::twist_msg_cb, this);
}

void
ROSCmdVelThread::twist_msg_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
  send_transrot(msg->linear.x, msg->linear.y, msg->angular.z);
}

bool
ROSCmdVelThread::prepare_finalize_user()
{
  stop();
  return true;
}

void
ROSCmdVelThread::finalize()
{
  blackboard->close(motor_if_);
  sub_.shutdown();
}

void
ROSCmdVelThread::send_transrot(float vx, float vy, float omega)
{
  if (motor_if_->has_writer()) {
    MotorInterface::TransRotMessage *msg =
      new MotorInterface::TransRotMessage(vx, vy, omega);
    motor_if_->msgq_enqueue(msg);
  } else {
    logger->log_warn(name(), "Cannot send transrot, no writer on motor interface");
  }
}

void
ROSCmdVelThread::stop()
{
  send_transrot(0., 0., 0.);
}

void
ROSCmdVelThread::loop()
{
}
