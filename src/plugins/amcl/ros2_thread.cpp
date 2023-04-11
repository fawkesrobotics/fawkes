/***************************************************************************
 *  ros_thread.cpp - Thread to interact with ROS for amcl plugin
 *
 *  Created: Mon Jun 22 17:50:24 2015
 *  Copyright  2012-2015  Tim Niemueller [www.niemueller.de]
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

/* Based on amcl_node.cpp from the ROS amcl package
 * Copyright (c) 2008, Willow Garage, Inc.
 */

#include "ros_thread.h"

#include <rclcpp/rclcpp.hpp>

using namespace fawkes;

/** @class AmclROS2Thread "ros_thread.h"
 * Thread for ROS integration of the Adaptive Monte Carlo Localization.
 * @author Tim Niemueller
 */

/** Constructor. */
AmclROS2Thread::AmclROS2Thread() : Thread("AmclROS2Thread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
AmclROS2Thread::~AmclROS2Thread()
{
}

void
AmclROS2Thread::init()
{
	pose_pub_          = node_handle->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 2);
	particlecloud_pub_ = node_handle->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 2);
	initial_pose_sub_ =
	  node_handle->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 2, std::bind(&AmclROS2Thread::initial_pose_received, this, std::placeholders::_1));
	map_pub_ = node_handle->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

	loc_if_ = blackboard->open_for_reading<LocalizationInterface>("AMCL");
}

void
AmclROS2Thread::finalize()
{
	blackboard->close(loc_if_);

// TODO: implement proper shutdown accordingly
//	pose_pub_.shutdown();
//	particlecloud_pub_.shutdown();
//	initial_pose_sub_.shutdown();
//	map_pub_.shutdown();
}

void
AmclROS2Thread::loop()
{
}

/** Publish pose array to ROS.
 * @param global_frame_id Name of the global frame
 * @param set sample set to publish
 */
void
AmclROS2Thread::publish_pose_array(const std::string &global_frame_id, const pf_sample_set_t *set)
{
	geometry_msgs::msg::PoseArray cloud_msg;
	cloud_msg.header.stamp    = node_handle->get_clock()->now();
	cloud_msg.header.frame_id = global_frame_id;
	cloud_msg.poses.resize(set->sample_count);
	for (int i = 0; i < set->sample_count; i++) {
		tf::Quaternion q(tf::create_quaternion_from_yaw(set->samples[i].pose.v[2]));
		cloud_msg.poses[i].position.x    = set->samples[i].pose.v[0];
		cloud_msg.poses[i].position.y    = set->samples[i].pose.v[1];
		cloud_msg.poses[i].position.z    = 0.;
		cloud_msg.poses[i].orientation.x = q.x();
		cloud_msg.poses[i].orientation.y = q.y();
		cloud_msg.poses[i].orientation.z = q.z();
		cloud_msg.poses[i].orientation.w = q.w();
	}

	particlecloud_pub_->publish(cloud_msg);
}

/** Publish pose with covariance to ROS.
 * @param global_frame_id Name of the global frame
 * @param amcl_hyp AMCL hypothesis to finish, i.e. the converged pose
 * @param covariance covariance associated with the pose
 */
void
AmclROS2Thread::publish_pose(const std::string &global_frame_id,
                             const amcl_hyp_t  &amcl_hyp,
                             const double       covariance[36])
{
	geometry_msgs::msg::PoseWithCovarianceStamped p;
	// Fill in the header
	p.header.frame_id = global_frame_id;
//	p.header.stamp    = ros::Time();
	p.header.stamp    = node_handle->get_clock()->now();
	// Copy in the pose
	p.pose.pose.position.x = amcl_hyp.pf_pose_mean.v[0];
	p.pose.pose.position.y = amcl_hyp.pf_pose_mean.v[1];
	tf::Quaternion q(tf::Vector3(0, 0, 1), amcl_hyp.pf_pose_mean.v[2]);
	p.pose.pose.orientation.x = q.x();
	p.pose.pose.orientation.y = q.y();
	p.pose.pose.orientation.z = q.z();
	p.pose.pose.orientation.w = q.w();

	// Copy in the covariance
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			// Report the overall filter covariance, rather than the
			// covariance for the highest-weight cluster
			p.pose.covariance[6 * i + j] = covariance[6 * i + j];
		}
	}
	p.pose.covariance[6 * 5 + 5] = covariance[6 * 5 + 5];

	pose_pub_->publish(p);
}

/** Publish map to ROS.
 * @param global_frame_id Name of the global frame
 * @param map map to publish
 */
void
AmclROS2Thread::publish_map(const std::string &global_frame_id, const map_t *map)
{
	nav_msgs::msg::OccupancyGrid msg;
	msg.info.map_load_time = node_handle->get_clock()->now();
	msg.header.stamp       = node_handle->get_clock()->now();
	msg.header.frame_id    = global_frame_id;

	msg.info.width             = map->size_x;
	msg.info.height            = map->size_y;
	msg.info.resolution        = map->scale;
	msg.info.origin.position.x = map->origin_x - (map->size_x / 2) * map->scale;
	msg.info.origin.position.y = map->origin_y - (map->size_y / 2) * map->scale;
	msg.info.origin.position.z = 0.0;
	tf::Quaternion q(tf::create_quaternion_from_yaw(0));
	msg.info.origin.orientation.x = q.x();
	msg.info.origin.orientation.y = q.y();
	msg.info.origin.orientation.z = q.z();
	msg.info.origin.orientation.w = q.w();

	// Allocate space to hold the data
	msg.data.resize((size_t)msg.info.width * msg.info.height);

	for (unsigned int i = 0; i < msg.info.width * msg.info.height; ++i) {
		if (map->cells[i].occ_state == +1) {
			msg.data[i] = +100;
		} else if (map->cells[i].occ_state == -1) {
			msg.data[i] = 0;
		} else {
			msg.data[i] = -1;
		}
	}

  logger->log_info(name(), "publish map");
	map_pub_->publish(msg);
}

void
AmclROS2Thread::initial_pose_received(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
	fawkes::Time msg_time(msg.header.stamp.sec, msg.header.stamp.nanosec / 1000);

  logger->log_info(name(), "pose received");
	const double *covariance    = msg.pose.covariance.data();
	const double  rotation[]    = {msg.pose.pose.orientation.x,
                             msg.pose.pose.orientation.y,
                             msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w};
	const double  translation[] = {msg.pose.pose.position.x,
                                msg.pose.pose.position.y,
                                msg.pose.pose.position.z};

	std::string frame = msg.header.frame_id;
	if (!frame.empty() && frame[0] == '/')
		frame = frame.substr(1);

	LocalizationInterface::SetInitialPoseMessage *ipm =
	  new LocalizationInterface::SetInitialPoseMessage();
	ipm->set_frame(frame.c_str());
	ipm->set_translation(translation);
	ipm->set_rotation(rotation);
	ipm->set_covariance(covariance);

	try {
		loc_if_->msgq_enqueue(ipm);
	} catch (Exception &e) {
		logger->log_error(name(), "Failed to set pose, exception follows");
		logger->log_error(name(), e);
	}
}
