
/***************************************************************************
 *  node_thread.cpp - Gazebo node handle providing Thread
 *
 *  Created: Fri Aug 24 11:04:04 2012
 *  Author  Bastian Klingen, Frederik Zwilling (2013)
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

#include "node_thread.h"

#include <gazebo/gazebo_config.h>
#include <google/protobuf/message.h>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportIface.hh>
#include <gazebo/transport/TransportTypes.hh>

using namespace fawkes;

/** @class GazeboNodeThread "node_thread.h"
 * Gazebo node handle thread.
 * This thread maintains a Gazebo node which can be used by other
 * threads and is provided via the GazeboAspect.
 *
 * @author Bastian Klingen, Frederik Zwilling
 */

/** Constructor. */
GazeboNodeThread::GazeboNodeThread()
: Thread("GazeboNodeThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP),
  AspectProviderAspect(&gazebo_aspect_inifin_)
{
}

/** Destructor. */
GazeboNodeThread::~GazeboNodeThread()
{
}

void
GazeboNodeThread::init()
{
	const std::string robot_channel =
	  config->get_string("/gazsim/world-name") + "/" + config->get_string("/gazsim/robot-name");

	logger->log_info(name(), "Robot channel: %s", robot_channel.c_str());
	const std::string world_name = config->get_string("/gazsim/world-name");

	if (!gazebo::client::setup()) {
		throw Exception("Failed to initialize Gazebo client");
	}

	// Initialize Communication nodes:
	// Global world node
	gazebo_world_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
	logger->log_info(name(), "Initializing world node to namespace '%s'", world_name.c_str());
	gazebo_world_node_->Init(world_name.c_str());
	gazebo_aspect_inifin_.set_gazebo_world_node(gazebo_world_node_);
	// Robot-specific node
	// This node only communicates with nodes that were initialized with the same string.
	gazebonode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
	logger->log_info(name(), "Initializing node to namespace '%s'", robot_channel.c_str());
	gazebonode_->Init(robot_channel);
	// For some reason, we need to advertise a topic to be connected to the master properly.
	status_publisher_ = gazebonode_->Advertise<gazebo::msgs::Time>("~/heartbeat");
	gazebo_aspect_inifin_.set_gazebonode(gazebonode_);
}

void
GazeboNodeThread::finalize()
{
	gazebonode_->Fini();
	gazebonode_.reset();
	gazebo_aspect_inifin_.set_gazebonode(gazebonode_);
	gazebo_world_node_->Fini();
	gazebo_world_node_.reset();
	gazebo_aspect_inifin_.set_gazebonode(gazebo_world_node_);
}

void
GazeboNodeThread::loop()
{
	gazebo::msgs::Time time;
	status_publisher_->Publish(time);
}
