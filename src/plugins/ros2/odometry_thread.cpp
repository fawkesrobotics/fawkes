/***************************************************************************
 *  odometry_thread.h - Thread to publish odometry to ROS
 *
 *  Created: Fri Jun 1 13:29:39 CEST
 *  Copyright  2012  Sebastian Reuter
 *             2017  Till Hofmann
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

#include "odometry_thread.h"

#include <interfaces/MotorInterface.h>
#include <tf/types.h>

using namespace fawkes;

/** @class ROS2OdometryThread "odometry_thread.h"
 * Thread to publish odometry to ROS.
 * @author Sebastian Reuter
 */

/** Constructor. */
ROS2OdometryThread::ROS2OdometryThread()
: Thread("OdometryThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP)
{
}

void
ROS2OdometryThread::init()
{
	std::string motor_if_id = config->get_string("/ros2/odometry/motor_interface_id");
	cfg_odom_frame_id_      = config->get_string("/ros2/odometry/odom_frame_id");
	cfg_base_frame_id_      = config->get_string("/ros2/odometry/base_frame_id");
	if (config->exists("/ros2/odometry/odom/covariance")) {
		odom_covariance_.assign(0.);
		std::vector<float> cfg_odom_covariance = config->get_floats("/ros2/odometry/odom/covariance");
		for (uint i = 0; i < cfg_odom_covariance.size() && i < odom_covariance_.size(); i++) {
			odom_covariance_[i] = cfg_odom_covariance.size();
		}
	} else {
		odom_covariance_ = {1e-3, 0., 0.,   0., 0.,   0., 0., 1e-3, 0., 0.,   0., 0.,
		                    0.,   0., 1e-3, 0., 0.,   0., 0., 0.,   0., 1e-3, 0., 0.,
		                    0.,   0., 0.,   0., 1e-3, 0., 0., 0.,   0., 0.,   0., 1e-3};
	}
	motor_if_ = blackboard->open_for_reading<MotorInterface>(motor_if_id.c_str());
	pub_      = node_handle->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  logger->log_info(name(), "done");
}

void
ROS2OdometryThread::finalize()
{
	blackboard->close(motor_if_);
}

void
ROS2OdometryThread::publish_odom()
{
	nav_msgs::msg::Odometry odom;
	//Header
	odom.header.stamp    = node_handle->get_clock()->now();
	odom.header.frame_id = cfg_odom_frame_id_;
	//Position
	odom.pose.pose.position.x = (double)motor_if_->odometry_position_x();
	odom.pose.pose.position.y = (double)motor_if_->odometry_position_y();
	odom.pose.pose.position.z = 0.0;
	std::copy(odom_covariance_.begin(), odom_covariance_.end() , std::begin(odom.pose.covariance));
	fawkes::tf::Quaternion    q(motor_if_->odometry_orientation(), 0, 0);
	odom.pose.pose.orientation.x                = q.x();
	odom.pose.pose.orientation.y                = q.y();
	odom.pose.pose.orientation.z                = q.z();
	odom.pose.pose.orientation.w                = q.w();
	//Motion
	odom.child_frame_id        = cfg_base_frame_id_;
	odom.twist.twist.linear.x  = (double)motor_if_->vx();
	odom.twist.twist.linear.y  = (double)motor_if_->vy();
	odom.twist.twist.angular.z = (double)motor_if_->omega();
	std::copy(odom_covariance_.begin(), odom_covariance_.end() , std::begin(odom.twist.covariance));
	pub_->publish(odom);
}

void
ROS2OdometryThread::loop()
{
	motor_if_->read();
	publish_odom();
}
