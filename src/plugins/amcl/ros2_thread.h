/***************************************************************************
 *  ros_thread.cpp - Thread to interact with ROS for amcl plugin
 *
 *  Created: Mon Jun 22 17:46:40 2015
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_AMCL_ROS_THREAD_H_
#define _PLUGINS_AMCL_ROS_THREAD_H_

//#ifndef HAVE_ROS
//#	error "ROS integration requires ROS support of system"
//#endif

#include "amcl_thread.h"
#include "map/map.h"
#include "pf/pf.h"

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <interfaces/LocalizationInterface.h>
#include <plugins/ros2/aspect/ros2.h>

namespace fawkes {
class Mutex;
}

class AmclThread;

class AmclROS2Thread : public fawkes::Thread,
                      public fawkes::LoggingAspect,
                      public fawkes::ConfigurableAspect,
                      public fawkes::BlackBoardAspect,
                      public fawkes::ROS2Aspect
{
public:
	AmclROS2Thread();
	virtual ~AmclROS2Thread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	void publish_pose_array(const std::string &global_frame_id, const pf_sample_set_t *set);
	void publish_pose(const std::string &global_frame_id,
	                  const amcl_hyp_t  &amcl_hyp,
	                  const double       last_covariance[36]);
	void publish_map(const std::string &global_frame_id, const map_t *map);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void initial_pose_received(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);

private:
	std::string cfg_pose_ifname_;

	fawkes::LocalizationInterface *loc_if_;

	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr  pose_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr  particlecloud_pub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr  initial_pose_sub_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr  map_pub_;
};

#endif
