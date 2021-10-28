/***************************************************************************
 *  robot_description_thread.cpp - ROS Robot Description Plugin
 *
 *  Created: Fri May 16 15:35:42 2014
 *  Copyright  2014  Till Hofmann
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

#include "robot_description_thread.h"

#include <rclcpp/rclcpp.hpp>

#include <fstream>

using namespace fawkes;
using namespace std;

#define CFG_PREFIX "/ros/robot-description/"

/** @class ROS2RobotDescriptionThread "robot_description_thread.h"
 * Thread to publish the robot description to ROS
 * @author Till Hofmann
 */

ROS2RobotDescriptionThread::ROS2RobotDescriptionThread()
: Thread("ROS2RobotDescriptionThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

ROS2RobotDescriptionThread::~ROS2RobotDescriptionThread()
{
}

void
ROS2RobotDescriptionThread::init()
{
	cfg_urdf_path_ = config->get_string(CFG_PREFIX "urdf_file");
	cfg_ros_param_ = config->get_string(CFG_PREFIX "ros_robot_description");

	string urdf;
	string line;
	if (cfg_urdf_path_.substr(0, 1) != "/") {
		// relative path, add prefix RESDIR/urdf/
		cfg_urdf_path_.insert(0, RESDIR "/urdf/");
	}
	ifstream urdf_file(cfg_urdf_path_.c_str());
	if (!urdf_file.is_open()) {
		logger->log_error(name(), "failed to open URDF File %s", cfg_urdf_path_.c_str());
		throw Exception("Failed to open URDF File %s", cfg_urdf_path_.c_str());
	}
	while (getline(urdf_file, line)) {
		urdf += line;
	}
	urdf_file.close();

	node_handle->set_parameter(rclcpp::Parameter(cfg_ros_param_, urdf));
}

void
ROS2RobotDescriptionThread::finalize()
{
	node_handle->set_parameter(rclcpp::Parameter(cfg_ros_param_, ""));
}
