/***************************************************************************
 *  laserbridge_thread.cpp - Takes ros2 scan topic and sends it to fawkes
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
#include "laserbridge_thread.h"

#include <cmath>
#include <mutex>

//This plugin injects the laser scan data from ROS2 into the fawkes pcl_manager
//Therefore the laser-data is first filterd like in the default config for laser-filter:
//i.e. remove all date from within the robotino and only keep those inside certain angles
//The plugin also calculates the minimum distance of a set of beams in the front of the robot
//and publishes it to the front-dist interface

using namespace fawkes;
ROS2LaserBridgeThread::ROS2LaserBridgeThread()
: Thread("LaserBridgeThread", Thread::OPMODE_WAITFORWAKEUP),
  TransformAspect(TransformAspect::ONLY_PUBLISHER, "ros2-laserbridge"),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
  ros2_cloud_()

{
}

ROS2LaserBridgeThread::~ROS2LaserBridgeThread()
{
}

void
ROS2LaserBridgeThread::init()
{
	std::string ros_namespace = config->get_string_or_default("/ros2/namespace", "");
	std::string scan_topic_name =
	  config->get_string_or_default("ros2/laserbridge/scan_topic_name", "/scan");

	laser_sub_ = node_handle->create_subscription<sensor_msgs::msg::LaserScan>(
	  "/" + ros_namespace + scan_topic_name,
	  10,
	  std::bind(&ROS2LaserBridgeThread::laser_callback, this, std::placeholders::_1));
	cloud_                  = new pcl::PointCloud<pcl::PointXYZ>();
	cloud_->width           = 0;
	cloud_->height          = 1;
	cloud_->header.frame_id = "front_laser";
	pcl_manager->add_pointcloud("front-filtered-1080", cloud_);

	// read config values
	beams_used_   = config->get_int("plugins/laser-front-dist/number_beams_used");
	target_frame_ = config->get_string("plugins/laser-front-dist/target_frame");

	if_front_dist_ = blackboard->open_for_writing<Position3DInterface>(
	  config->get_string("plugins/laser-front-dist/output_result_interface").c_str());
}

#define INCREMENT 2 * M_PI / 1080

void
ROS2LaserBridgeThread::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
	std::lock_guard lock(main_loop);
	// Assuming the laser scanner is horizontally mounted and scan is in the x-y plane
	if (scan->ranges.size() == 0)
		return;
	if (scan->ranges.size() != ros2_cloud_.size()) {
		ros2_cloud_.points.resize(scan->ranges.size());
		ros2_cloud_.height = 1;
		ros2_cloud_.width  = scan->ranges.size();
	}
	for (unsigned int i = 0; i < scan->ranges.size(); ++i) {
		pcl::PointXYZ &point = ros2_cloud_.points[i];
		if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
			point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
			continue;
		}

		float range = scan->ranges[i];
		float angle = scan->angle_min + i * scan->angle_increment;
		if (angle > 335 * INCREMENT && angle < 765 * INCREMENT) { //LEGACY
			point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
			continue;
		}

		point.x = range * cos(angle);
		point.y = range * sin(angle);
		point.z = 0; // Since this is a 2D laser scan, z will be 0
	}

	int center_beam_index = scan->ranges.size() / 2;

	float min = std::numeric_limits<float>::infinity();

	for (int i = center_beam_index - beams_used_ / 2; i < center_beam_index + beams_used_ / 2; i++) {
		if (!std::isnormal(scan->ranges[i])) {
			// this is invalid
			if_front_dist_->set_visibility_history(-1);
			if_front_dist_->write();
			return;
		}
		if (min > scan->ranges[i]) {
			min = scan->ranges[i];
		}
	}

	std::string frame_ = "front_laser";

	tf::Transform        transform(tf::create_quaternion_from_yaw(M_PI), tf::Vector3(min, 0, 0));
	Time                 time(clock);
	tf::StampedTransform stamped_transform(transform, time, frame_.c_str(), target_frame_.c_str());
	tf_publisher->send_transform(stamped_transform);

	// write result
	if_front_dist_->set_visibility_history(1);
	if_front_dist_->set_translation(0, min);
	if_front_dist_->set_frame(frame_.c_str());
	if_front_dist_->write();
}

void
ROS2LaserBridgeThread::loop()
{
	std::lock_guard lock(main_loop);
	if (cloud_->size() != ros2_cloud_.size()) {
		cloud_->points.resize(ros2_cloud_.size());
		cloud_->height = 1;
		cloud_->width  = ros2_cloud_.size();
	}
	for (unsigned int i = 0; i < cloud_->width; ++i) {
		cloud_->points[i] = ros2_cloud_[i];
	}
}
void
ROS2LaserBridgeThread::finalize()
{
}
