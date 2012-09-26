/***************************************************************************
 *  odometry_thread.h - Thread to publish odometry to ROS
 *
 *  Created: Fri Jun 1 13:29:39 CEST
 *  Copyright  2012  Sebastian Reuter
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

#include "odometry_thread.h"
#include <interfaces/MotorInterface.h>
#include <nav_msgs/Odometry.h>
#include <tf/types.h>

using namespace fawkes;

/** @class PluginTemplateThread "plugin_template_thread.h"
 * Introductional example thread which makes the robotino drive along a 8-shaped course
 * @author Daniel Ewert
 */

/** Constructor. */
ROSOdometryThread::ROSOdometryThread() :
		Thread("OdometryThread", Thread::OPMODE_WAITFORWAKEUP), BlockedTimingAspect(
				BlockedTimingAspect::WAKEUP_HOOK_SKILL) {
}

void  ROSOdometryThread::init() {

	logger->log_info(name(), "Odometry starts up");
	//Blackboard
	motor_if_ = blackboard->open_for_reading<MotorInterface>("Robotino");
	//ROS
	__pub = rosnode->advertise<nav_msgs::Odometry>("odom", 100, this);
}

bool  ROSOdometryThread::prepare_finalize_user() {
	return true;
}

void  ROSOdometryThread::finalize() {
	blackboard->close(motor_if_);
	__pub.shutdown();
}

nav_msgs::Odometry  ROSOdometryThread::createOdometryMessage() {
	//Odometry Message
	nav_msgs::Odometry odoMsg;
	//Header
	odoMsg.header.stamp = ros::Time::now();
	odoMsg.header.frame_id = "robotino_odometry";
	//Position
	odoMsg.pose.pose.position.x = (double)motor_if_->odometry_position_x();
	odoMsg.pose.pose.position.y = (double) motor_if_->odometry_position_y();
	odoMsg.pose.pose.position.z = 0.0;
	fawkes::tf::Quaternion q(motor_if_->odometry_orientation(), 0, 0);
	geometry_msgs::Quaternion odom_quat;
	odom_quat.x = q.x();
	odom_quat.y = q.y();
	odom_quat.z = q.z();
	odom_quat.w = q.w();
	odoMsg.pose.pose.orientation = odom_quat;
	//Bewegung
	odoMsg.child_frame_id = "base_link";
	odoMsg.twist.twist.linear.x = (double)motor_if_->vx();
	odoMsg.twist.twist.linear.y = (double)motor_if_->vy();
	odoMsg.twist.twist.angular.z = (double)motor_if_->omega();
	return odoMsg;
}

void  ROSOdometryThread::sendOdometryToROS() {
	//Odometry Message
	//logger->log_info(name(),"erstelle Odometry Message");
	nav_msgs::Odometry odoMsg = createOdometryMessage();
	//sende Daten an ROS
	//logger->log_info(name(), "schicke Daten an ROS");
	__pub.publish(odoMsg);
}

void  ROSOdometryThread::loop() {
	//alles aktualisieren
	//logger->log_info(name(), "aktualisiere Interfaces");
	motor_if_->read();
	sendOdometryToROS();
}

