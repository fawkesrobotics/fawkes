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

#ifndef _PLUGINS_ROS2_CMDVEL_THREAD_H_
#define _PLUGINS_ROS2_CMDVEL_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <geometry_msgs/msg/twist.hpp>
#include <plugins/ros2/aspect/ros2.h>
#include <rclcpp/rclcpp.hpp>

namespace fawkes {
class MotorInterface;
}

class ROS2CmdVelThread : public fawkes::Thread,
                        public fawkes::LoggingAspect,
                        public fawkes::ConfigurableAspect,
                        public fawkes::BlackBoardAspect,
                        public fawkes::ROS2Aspect
{
public:
	ROS2CmdVelThread();

	virtual void init();
	virtual void loop();
	virtual bool prepare_finalize_user();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void stop();                                         //stops all Motors
	void send_transrot(float vx, float vy, float omega) const; //sends Controls to the Motors
	void twist_msg_cb(const geometry_msgs::msg::Twist::SharedPtr msg) const;

private:
	fawkes::MotorInterface *motor_if_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr	 sub_;
	
};

#endif
