
/***************************************************************************
 *  laserscan_thread.cpp - Thread to exchange IR Sensor data
 *
 *  Created: Mon Jul 03 13:41:18 2012
 *  Copyright  2023 Saurabh Borse, Tim Wendt
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

#ifndef _PLUGINS_ROS_IRSCANNER_THREAD_H_
#define _PLUGINS_ROS_IRSCANNER_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/RobotinoSensorInterface.h>
#include <plugins/ros2/aspect/ros2.h>
#include <utils/time/time.h>

#include <list>
#include <memory>
#include <queue>
using std::placeholders::_1;

//ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ROS2IrScanThread : public fawkes::Thread,
                         public fawkes::ConfigurableAspect,
                         public fawkes::BlockedTimingAspect,
                         public fawkes::BlackBoardAspect,
                         public fawkes::ROS2Aspect,
                         public fawkes::BlackBoardInterfaceListener
{
public:
	ROS2IrScanThread();

	virtual void init();
	virtual void finalize();
	virtual void loop();

protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	fawkes::RobotinoSensorInterface *sens_if_ = nullptr;

	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
	sensor_msgs::msg::LaserScan                               msg;
};

#endif
