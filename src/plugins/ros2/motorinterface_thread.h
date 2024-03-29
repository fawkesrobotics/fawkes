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

#ifndef _PLUGINS_ROS2_MOTORINTERFACE_THREAD_H_
#define _PLUGINS_ROS2_MOTORINTERFACE_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <plugins/ros2/aspect/ros2.h>
#include <blackboard/interface_listener.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fawkes {
class MotorInterface;
}

class ROS2MotorInterfaceThread : public fawkes::Thread,
                         public fawkes::LoggingAspect,
                         public fawkes::ConfigurableAspect,
                         public fawkes::BlackBoardAspect,
                         public fawkes::BlackBoardInterfaceListener,
                         public fawkes::ClockAspect,
                         public fawkes::ROS2Aspect
{
public:
	ROS2MotorInterfaceThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	virtual bool bb_interface_message_received(fawkes::Interface *interface,
	                                           fawkes::Message   *message) throw();
	fawkes::MotorInterface                                    *motor_if_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

#endif
