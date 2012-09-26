/***************************************************************************
 *  cmdvel_plugin.cpp - Translate ROS Twist messages to Navgiator transrot
 *
 *  Created: Fri Jun 1 13:29:39 CEST 2012
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

#include "cmdvel_thread.h"
#include <interfaces/MotorInterface.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>

//using namespace ros;
using namespace fawkes;

/** @class PluginTemplateThread "plugin_template_thread.h"
 * Introductional example thread which makes the robotino drive along a 8-shaped course
 * @author Daniel Ewert
 */

/** Constructor. */
ROSCmdVelThread::ROSCmdVelThread() :
		Thread("ROSCmdVelThread", Thread::OPMODE_WAITFORWAKEUP), BlockedTimingAspect(
				BlockedTimingAspect::WAKEUP_HOOK_SKILL) {
}

void ROSCmdVelThread::init() {

	//logger->log_info(name(), "ros_cmdvel starts up");
	//Blackboard
	motor_if_ = blackboard->open_for_reading<MotorInterface>("Robotino");
	//ROS
	__sub = rosnode->subscribe("cmd_vel",100,&ROSCmdVelThread::callback,this);
}

void ROSCmdVelThread::callback(const geometry_msgs::Twist::ConstPtr &msg)
{
	//logger->log_info(name(),"bekomme Daten von ROS");
	//float vx = msg->linear.x;
	//float vy = msg->linear.y;
	//float omega = msg->angular.z;
	send_transrot(msg->linear.x,msg->linear.y,msg->angular.z);
}

bool ROSCmdVelThread::prepare_finalize_user() {
	stop();
	return true;
}

void ROSCmdVelThread::finalize() {
	blackboard->close(motor_if_);
	__sub.shutdown();
}

void ROSCmdVelThread::send_transrot(float vx, float vy, float omega) {
	MotorInterface::TransRotMessage *msg = new MotorInterface::TransRotMessage(
			vx, vy, omega);
	(*motor_if_).msgq_enqueue(msg);
}

void ROSCmdVelThread::stop() {
	send_transrot(0., 0., 0.);
}

void ROSCmdVelThread::loop() {
	//alles aktualisieren
	//logger->log_info(name(), "Spin");
	//ros::spinOnce();
}

