/***************************************************************************
 *
 *  laserbridge_thread.h - Takes ros2 scan topic and sends it to fawkes
 *  pcl_manager
 *
 *  Created: Tue March 30 19:30:47 2014
 *  Copyright  2024  Tim Wendt
 *
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

#ifndef _PLUGINS_ROS2_LASER_BRIDGE_H_
#define _PLUGINS_ROS2_LASER_BRIDGE_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/pointcloud.h>
#include <aspect/tf.h>
#include <core/threading/thread.h>
#include <core/utils/refptr.h>
#include <interfaces/Position3DInterface.h>
#include <plugins/ros2/aspect/ros2.h>
#include <tf/types.h>

#include <mutex>
#include <sensor_msgs/msg/laser_scan.hpp>

class ROS2LaserBridgeThread : public fawkes::Thread,
                              public fawkes::ROS2Aspect,
                              public fawkes::ClockAspect,
                              public fawkes::ConfigurableAspect,
                              public fawkes::TransformAspect,
                              public fawkes::BlockedTimingAspect,
                              public fawkes::BlackBoardAspect,
                              public fawkes::PointCloudAspect
{
public:
	ROS2LaserBridgeThread();
	virtual ~ROS2LaserBridgeThread();

	virtual void init();
	virtual void finalize();
	virtual void loop();

private:
	std::mutex main_loop;
	void       laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

	fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZ>> cloud_;
	pcl::PointCloud<pcl::PointXYZ>                 ros2_cloud_;

	std::mutex mutex_;

	int                          beams_used_;
	std::string                  target_frame_;
	fawkes::Position3DInterface *if_front_dist_;
};

#endif // _PLUGINS_ROS2_LASER_BRIDGE_H_
