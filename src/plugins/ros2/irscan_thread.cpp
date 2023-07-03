
/***************************************************************************
 *  laserscan_thread.cpp - Thread to exchange IR Sensor data
 *
 *  Created: Mon Jul 03 13:41:18 2012
 *  Copyright  2023  Shantanu Chivate, Tim Wendt
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

#include "irscan_thread.h"

#include <interfaces/RobotinoSensorInterface.h>
#include <limits>

using namespace fawkes;

/** @class ROSirScanThread "pcl_thread.h"
 * Thread to exchange ir sensor data between Fawkes and ROS.
 * @author Tim Niemueller
 */

/** Constructor. */
ROS2IrScanThread::ROS2IrScanThread()
: Thread("ROS2IrScanThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
  BlackBoardInterfaceListener("ROS2IrScanThread")
{
}

void
ROS2IrScanThread::init()
{
	sens_if_= blackboard->open_for_reading<RobotinoSensorInterface>("Robotino");

	sens_if_->read();

	std::string topname = "IrSensor_Scan";

	pi.pub = node_handle->create_publisher<sensor_msgs::msg::LaserScan>(topname, 1);

	pi.msg.header.frame_id = config->get_string("/hardware/robotino/base_frame");
	pi.msg.angle_min       = 0;
	pi.msg.angle_max       = 2 * M_PI;
	pi.msg.angle_increment = 0.68;
	pi.msg.scan_time = 0.1;
	pi.msg.range_min = 0.02;
	pi.msg.range_max = 0.12;

	blackboard->register_listener(this);
}


void
ROS2IrScanThread::finalize()
{
	blackboard->unregister_listener(this);
	blackboard->close(sens_if_);
}


void
ROS2IrScanThread::bb_interface_data_refreshed(fawkes::Interface *interface) noexcept
{
	RobotinoSensorInterface  *sensor_data_msg  = dynamic_cast<RobotinoSensorInterface *>(interface);
	if(!sensor_data_msg)
	return;

	sensor_data_msg->read();

	PublisherInfo               &pi  = pubs_[interface->uid()];
	sensor_msgs::msg::LaserScan &msg = pi.msg;
	const Time *time = sensor_data_msg->timestamp();
	msg.header.stamp    = rclcpp::Time(time->get_sec(), time->get_nsec());
	memcpy(&msg.ranges[0], sensor_data_msg->distance(), 9 * sizeof(float));

	pi.pub->publish(pi.msg);
	
}
