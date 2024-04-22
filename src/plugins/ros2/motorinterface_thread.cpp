
/***************************************************************************
 *  motorinterface_thread.cpp - Translates transrot motor messages to ROS2 Twist
 *
 *  Created: Sun Mar 31 13:29:39 CEST 2024
 *  Copyright  2024 Tim Wendt
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

using namespace fawkes;
/** @class ROS2MotorInterfaceThread "MotorInterface_thread.h"
 * Thread to Translates transrot motor messages to ROS2 Twist
 * @author Gjorgji Nikolovski
 */

/** Constructor. */
ROS2MotorInterfaceThread::ROS2MotorInterfaceThread()
: Thread("ROS2MotorInterfaceThread", Thread::OPMODE_CONTINUOUS),
  BlackBoardInterfaceListener("ROS2MotorInterfaceThread")
{
}

void
ROS2MotorInterfaceThread::init()
{
	std::string motor_if_id = config->get_string("/ros2/cmdvel/motor_interface_id");
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
                                                        fawkes::Message   *message) throw()
{
	if (message->is_of_type<MotorInterface::TransRotMessage>()) {
		MotorInterface::TransRotMessage *msg = (MotorInterface::TransRotMessage *)message;

		auto cmd      = geometry_msgs::msg::Twist();
		cmd.linear.x  = msg->vx();
		cmd.linear.y  = msg->vy();
		cmd.angular.z = msg->omega();
		pub_->publish(cmd);
		return true;
	}

	if (message->is_of_type<MotorInterface::AcquireControlMessage>()) {
		MotorInterface::AcquireControlMessage *msg = (MotorInterface::AcquireControlMessage *)message;

		motor_if_->set_controller(msg->controller());
		motor_if_->set_controller_thread_name(msg->controller_thread_name());
		motor_if_->write();

		return true;
	}

	logger->log_warn(name(), "Received unknown message type on motor interface");
	return false;
}

void
ROS2MotorInterfaceThread::loop()
{
}
