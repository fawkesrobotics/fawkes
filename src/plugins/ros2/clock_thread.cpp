/***************************************************************************
 *  clock_thread.h - Thread to publish clock to ROS
 *
 *  Created: Sun Jul 12 16:14:41 2015
 *  Copyright  2015  Tim Niemueller
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

#include "clock_thread.h"

#include <utils/time/wait.h>

using namespace fawkes;

/** @class ROS2ClockThread "clock_thread.h"
 * Thread to publish clock to ROS.
 * @author Tim Niemueller
 */

/** Constructor. */
ROS2ClockThread::ROS2ClockThread() : Thread("ClockThread", Thread::OPMODE_CONTINUOUS)
{
	set_prepfin_conc_loop(true);
}

void
ROS2ClockThread::init()
{
	cfg_freq_ = config->get_uint("/ros/clock/frequency");
	pub_      = node_handle->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
	node_handle->declare_parameter("/use_sim_time");
	node_handle->set_parameter(rclcpp::Parameter("/use_sim_time", rclcpp::ParameterValue(true)));

	set_local_ = node_handle->get_clock()->get_clock_type() == rcl_clock_type_t::RCL_SYSTEM_TIME;
	if (set_local_) {
		//find alternative
	}

	time_wait_ = new TimeWait(clock, 1000000l / cfg_freq_);
}

void
ROS2ClockThread::finalize()
{
	rclcpp::shutdown();
	delete time_wait_;
}

void
ROS2ClockThread::publish_clock()
{
	rosgraph_msgs::msg::Clock clock_msg;

	fawkes::Time now(clock);
	clock_msg.clock.sec  = now.get_sec();
	clock_msg.clock.nanosec = now.get_usec() * 1000;

	pub_->publish(clock_msg);

	if (set_local_) {
		//ros::Time::setNow(clock_msg.clock);
	}
}

void
ros_clock_cb(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
	//ros::Time::setNow(msg->clock);
}

void
ROS2ClockThread::loop()
{
	time_wait_->mark_start();
	publish_clock();
	time_wait_->wait_systime();
}
