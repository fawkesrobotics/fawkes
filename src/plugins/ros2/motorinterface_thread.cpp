/***************************************************************************
 *  motorinterface_plugin.cpp - Translate ROS2 Twist messages to Navgiator transrot
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

#include "motorinterface_thread.h"

#include <interfaces/MotorInterface.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

//using namespace ros;
using namespace fawkes;
/** @class ROS2MotorInterfaceThread "MotorInterface_thread.h"
 * Thread to translate ROS2 twist messages to navigator transrot messages.
 * @author Gjorgji Nikolovski
 */

/** Constructor. */
ROS2MotorInterfaceThread::ROS2MotorInterfaceThread() : Thread("ROS2MotorInterfaceThread", Thread::OPMODE_WAITFORWAKEUP),
																											BlackBoardInterfaceListener("ROS2MotorInterfaceThread")
{
}

void
ROS2MotorInterfaceThread::init()
{
	std::string motor_if_id = config->get_string("/ros2/MotorInterface/motor_interface_id");
	motor_if_               = blackboard->open_for_writing<MotorInterface>(motor_if_id.c_str());
	pub_                    = node_handle->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
	bbil_add_message_interface(motor_if_);

	blackboard->register_listener(this);
}

void
ROS2MotorInterfaceThread::finalize()
{
	blackboard->close(motor_if_);
}

bool
ROS2MotorInterfaceThread::bb_interface_message_received(fawkes::Interface *interface,
																					 fawkes::Message   *message) throw() {
	if (!message->is_of_type<MotorInterface::TransRotMessage>()) {
		logger->log_warn(name(), "Received unknown message type on motor interface");
		return false;
	}

	MotorInterface::TransRotMessage *msg = (MotorInterface::TransRotMessage *)message;

	auto cmd = geometry_msgs::msg::Twist();
	cmd.linear.x  = msg->vx();
	cmd.linear.y  = msg->vy();
	cmd.angular.z = msg->omega();
	pub_->publish(cmd);
	return true;
}

void
ROS2MotorInterfaceThread::loop()
{
}
