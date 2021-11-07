/***************************************************************************
 *  position_3d_thread.cpp - Publish 3D Position to ROS
 *
 *  Created: Wed Jul 16 17:04:42 2014
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

#include "position_3d_thread.h"


using namespace fawkes;

#define CFG_PREFIX "/ros/position-3d/"

/** @class ROS2Position3DThread "position_3d_thread.h"
 * Thread to publish Position3Ds to ROS.
 * This thread reads all Position3D Blackboard Interfaces and publishes every
 * position to ROS.
 * @author Till Hofmann
 */

/** Constructor. */
ROS2Position3DThread::ROS2Position3DThread()
: Thread("ROS2Position3DThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("ROS2Position3DThread")
{
}

/** Destructor. */
ROS2Position3DThread::~ROS2Position3DThread()
{
}

void
ROS2Position3DThread::init()
{
	cfg_ros_topic_ = "/objects";
	try {
		cfg_ros_topic_ = config->get_string(CFG_PREFIX "ros_topic");
	} catch (Exception &e) {
	} // use default

	ros2_pub_ = node_handle->create_publisher<fawkes_msgs::msg::Position3D>(cfg_ros_topic_, 100);
	// check for open Position3DInterfaces
	ifs_ = blackboard->open_multiple_for_reading<Position3DInterface>();
	for (std::list<Position3DInterface *>::iterator it = ifs_.begin(); it != ifs_.end(); ++it) {
		bbil_add_data_interface(*it);
	}
	// watch for creation of new Position3DInterfaces
	bbio_add_observed_create("Position3DInterface");

	// register to blackboard
	blackboard->register_listener(this);
	blackboard->register_observer(this);
}

void
ROS2Position3DThread::finalize()
{
	blackboard->unregister_listener(this);
	blackboard->unregister_observer(this);
	for (std::list<Position3DInterface *>::iterator it = ifs_.begin(); it != ifs_.end(); ++it) {
		blackboard->close(*it);
	}
}

void
ROS2Position3DThread::bb_interface_created(const char *type, const char *id) throw()
{
	if (strncmp(type, "Position3DInterface", INTERFACE_TYPE_SIZE_) != 0)
		return;
	Position3DInterface *interface;
	try {
		interface = blackboard->open_for_reading<Position3DInterface>(id);
	} catch (Exception &e) {
		logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
		return;
	}
	try {
		bbil_add_data_interface(interface);
		blackboard->update_listener(this);
		ifs_.push_back(interface);
	} catch (Exception &e) {
		blackboard->close(interface);
		logger->log_warn(name(), "Failed to register for %s:%s: %s", type, id, e.what());
		return;
	}
}

void
ROS2Position3DThread::bb_interface_writer_removed(Interface *  interface,
                                                 unsigned int instance_serial) throw()
{
	conditional_close(interface);
}

void
ROS2Position3DThread::bb_interface_reader_removed(Interface *  interface,
                                                 unsigned int instance_serial) throw()
{
	conditional_close(interface);
}

void
ROS2Position3DThread::conditional_close(Interface *interface) throw()
{
	// Verify it's a Position3DInterface
	Position3DInterface *iface = dynamic_cast<Position3DInterface *>(interface);
	if (!iface)
		return;

	std::list<Position3DInterface *>::iterator it;
	for (it = ifs_.begin(); it != ifs_.end(); ++it) {
		if (*interface == **it) {
			if (!interface->has_writer() && (interface->num_readers() == 1)) {
				// It's only us
				bbil_remove_data_interface(*it);
				blackboard->update_listener(this);
				blackboard->close(*it);
				ifs_.erase(it);
				break;
			}
		}
	}
}

void
ROS2Position3DThread::bb_interface_data_refreshed(Interface *interface) throw()
{
	Position3DInterface *iface = dynamic_cast<Position3DInterface *>(interface);
	if (!iface)
		return;
	iface->read();
	fawkes_msgs::msg::Position3D position;
	position.header.frame_id    = iface->frame();
	position.name               = iface->id();
	position.pose.position.x    = iface->translation()[0];
	position.pose.position.y    = iface->translation()[1];
	position.pose.position.z    = iface->translation()[2];
	position.pose.orientation.x = iface->rotation()[0];
	position.pose.orientation.y = iface->rotation()[1];
	position.pose.orientation.z = iface->rotation()[2];
	position.pose.orientation.w = iface->rotation()[3];
	position.visibility_history = iface->visibility_history();
	ros2_pub_->publish(position);
}
