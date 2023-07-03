
/***************************************************************************
 *  laserscan_thread.h - Thread to exchange laser scans
 *
 *  Created: Tue May 29 19:32:39 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_ROS_LASERSCAN_THREAD_H_
#define _PLUGINS_ROS_LASERSCAN_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/threading/mutex.h>
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
                            public fawkes::LoggingAspect,
                            public fawkes::ConfigurableAspect,
                            public fawkes::BlockedTimingAspect,
                            public fawkes::BlackBoardAspect,
                            public fawkes::ROS2Aspect,
                            public fawkes::BlackBoardInterfaceObserver,
                            public fawkes::BlackBoardInterfaceListener
{
public:
	ROS2IrScanThread();
	virtual ~ROS2IrScanThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();
	// for BlackBoardInterfaceListener
	void bb_interface_data_refreshed(fawkes::Interface *interface) noexcept override;
	
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	
	fawkes::RobotinoSensorInterface *sens_if_;
	
	/// @cond INTERNALS
	typedef struct
	{
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
		sensor_msgs::msg::LaserScan                               msg;
	} PublisherInfo;
	/// @endcond
	PublisherInfo pi;
	std::map<std::string, PublisherInfo> pubs_;

};

#endif
