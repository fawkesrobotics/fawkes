/***************************************************************************
 *  cmdvel_plugin.cpp - Translate ROS2 Twist messages to Navgiator transrot
 *
 *  Created: Fri Jun 1 13:29:39 CEST 2012
 *  Copyright  2021 Gjorgji Nikolovski
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

#include <geometry_msgs/msg/twist.hpp>
#include <interfaces/MotorInterface.h>
#include <rclcpp/rclcpp.hpp>

//using namespace ros;
using namespace fawkes;
using std::placeholders::_1;
/** @class ROS2CmdVelThread "cmdvel_thread.h"
 * Thread to translate ROS2 twist messages to navigator transrot messages.
 * @author Gjorgji Nikolovski
 */

/** Constructor. */
ROS2CmdVelThread::ROS2CmdVelThread() : Thread("ROS2CmdVelThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

void
ROS2CmdVelThread::init()
{
	std::string motor_if_id = config->get_string("/ros2/cmdvel/motor_interface_id");
	motor_if_               = blackboard->open_for_reading<MotorInterface>(motor_if_id.c_str());
	sub_                    = node_handle->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&ROS2CmdVelThread::twist_msg_cb, this, _1));
}

void
ROS2CmdVelThread::twist_msg_cb(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
	this->send_transrot(msg->linear.x, msg->linear.y, msg->angular.z);
}

bool
ROS2CmdVelThread::prepare_finalize_user()
{
	stop();
	return true;
}

void
ROS2CmdVelThread::finalize()
{
	blackboard->close(motor_if_);
}

void
ROS2CmdVelThread::send_transrot(float vx, float vy, float omega) const
{
	if (motor_if_->has_writer()) {
		MotorInterface::TransRotMessage *msg = new MotorInterface::TransRotMessage(vx, vy, omega);
		motor_if_->msgq_enqueue(msg);
	} else {
		logger->log_warn(name(), "Cannot send transrot, no writer on motor interface");
	}
}

void
ROS2CmdVelThread::stop()
{
	send_transrot(0., 0., 0.);
}

void
ROS2CmdVelThread::loop()
{
}
