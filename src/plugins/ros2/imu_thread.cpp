/***************************************************************************
 *  imu_thread.cpp - Thread to publish IMU data to ROS
 *
 *  Created: Mon 03 Apr 2017 12:41:33 CEST 12:41
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "imu_thread.h"

#include <interface/interface_info.h>

using namespace fawkes;

/** @class ROS2IMUThread "imu_thread.h"
 * Thread to publish IMU data to ROS.
 * This thread reads data from the IMU blackboard interface and publishes the
 * data to ROS.
 * @author Till Hofmann
 */

/** Constructor. */
ROS2IMUThread::ROS2IMUThread()
: Thread("ROS2IMUThread", Thread::OPMODE_WAITFORWAKEUP), BlackBoardInterfaceListener("ROS2IMUThread")
{
}

/** Destructor. */
ROS2IMUThread::~ROS2IMUThread()
{
}

/** Initialize the thread.
 * Open the blackboard interface and advertise the ROS topic.
 * Register as listener for the blackboard interface.
 */
void
ROS2IMUThread::init()
{
	std::string iface_name;
	try {
		iface_name = config->get_string("/ros/imu/interface");
	} catch (Exception &e) {
		InterfaceInfoList *imu_ifaces = blackboard->list("IMUInterface", "*");
		if (imu_ifaces->empty()) {
			logger->log_error(name(), "Cannot use default IMUInterface: No IMUInterface found!");
			throw;
		} else {
			iface_name = imu_ifaces->front().id();
			logger->log_info(name(),
			                 "%s. Using first IMU interface '%s'.",
			                 e.what_no_backtrace(),
			                 iface_name.c_str());
		}
	}
	std::string ros2_topic = "/imu/data";
	try {
		ros2_topic = config->get_string("/ros/imu/topic");
	} catch (Exception &e) {
		// ignore, use default
	}
	logger->log_info(name(),
	                 "Publishing IMU '%s' to ROS topic '%s'.",
	                 iface_name.c_str(),
	                 ros2_topic.c_str());
	ros2_pub_ = node_handle->create_publisher<sensor_msgs::msg::Imu>(ros2_topic, 100);

	iface_ = blackboard->open_for_reading<IMUInterface>(iface_name.c_str());
	bbil_add_data_interface(iface_);
	blackboard->register_listener(this);
}

/** Close all interfaces and ROS handles. */
void
ROS2IMUThread::finalize()
{
	blackboard->unregister_listener(this);
	blackboard->close(iface_);
}

/** Handle interface changes.
 * If the IMU interface changes, publish updated data to ROS.
 * @param interface interface instance that you supplied to bbil_add_data_interface()
 */
void
ROS2IMUThread::bb_interface_data_refreshed(Interface *interface) throw()
{
	IMUInterface *imu_iface = dynamic_cast<IMUInterface *>(interface);
	if (!imu_iface)
		return;
	imu_iface->read();
	sensor_msgs::msg::Imu ros_imu;
	ros_imu.header.frame_id = imu_iface->frame();
	ros_imu.orientation.x   = imu_iface->orientation()[0];
	ros_imu.orientation.y   = imu_iface->orientation()[1];
	ros_imu.orientation.z   = imu_iface->orientation()[2];
	ros_imu.orientation.w   = imu_iface->orientation()[3];
	for (uint i = 0; i < 9u; i++) {
		ros_imu.orientation_covariance[i] = imu_iface->orientation_covariance()[i];
	}
	ros_imu.angular_velocity.x = imu_iface->angular_velocity()[0];
	ros_imu.angular_velocity.y = imu_iface->angular_velocity()[1];
	ros_imu.angular_velocity.z = imu_iface->angular_velocity()[2];
	for (uint i = 0; i < 9u; i++) {
		ros_imu.angular_velocity_covariance[i] = imu_iface->angular_velocity_covariance()[i];
	}
	ros_imu.linear_acceleration.x = imu_iface->linear_acceleration()[0];
	ros_imu.linear_acceleration.y = imu_iface->linear_acceleration()[1];
	ros_imu.linear_acceleration.z = imu_iface->linear_acceleration()[2];
	for (uint i = 0; i < 9u; i++) {
		ros_imu.linear_acceleration_covariance[i] = imu_iface->linear_acceleration_covariance()[i];
	}
	ros2_pub_->publish(ros_imu);
}
