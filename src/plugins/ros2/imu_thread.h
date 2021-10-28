/***************************************************************************
 *  imu_thread.h - Thread to publish IMU data to ROS
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

#ifndef _PLUGINS_ROS2_IMU_THREAD_H_
#define _PLUGINS_ROS2_IMU_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/IMUInterface.h>
#include <plugins/ros2/aspect/ros2.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
class ROS2IMUThread : public fawkes::Thread,
                     public fawkes::ConfigurableAspect,
                     public fawkes::LoggingAspect,
                     public fawkes::ROS2Aspect,
                     public fawkes::BlackBoardAspect,
                     public fawkes::BlackBoardInterfaceListener
{
public:
	ROS2IMUThread();
	virtual ~ROS2IMUThread();

	virtual void init();
	virtual void finalize();

	virtual void bb_interface_data_refreshed(fawkes::Interface *interface) throw();

private:
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr        ros2_pub_;
	fawkes::IMUInterface *iface_;
};

#endif /* PLUGINS_ROS_IMU_THREAD_H__ */
