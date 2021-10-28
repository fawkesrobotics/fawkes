/***************************************************************************
 *  odometry_thread.h - Thread to publish odometry to ROS
 *
 *  Created: Fri Jun 1 13:29:39 CEST
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

#ifndef _PLUGINS_ROS_ODOMETRY_THREAD_H_
#define _PLUGINS_ROS_ODOMETRY_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <plugins/ros2/aspect/ros2.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <boost/array.hpp>
namespace fawkes {
class MotorInterface;
}

class ROS2OdometryThread : public fawkes::Thread,
                          public fawkes::BlockedTimingAspect,
                          public fawkes::LoggingAspect,
                          public fawkes::ConfigurableAspect,
                          public fawkes::BlackBoardAspect,
                          public fawkes::ROS2Aspect
{
public:
	ROS2OdometryThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void publish_odom();

private:
	fawkes::MotorInterface * motor_if_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
	std::string              cfg_odom_frame_id_;
	std::string              cfg_base_frame_id_;
	boost::array<double, 36> odom_covariance_;
};

#endif
