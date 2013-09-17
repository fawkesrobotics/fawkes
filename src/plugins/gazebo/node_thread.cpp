
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

// from Gazebo
#include <gazebo/transport/TransportIface.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/gazebo_config.h>
#include <google/protobuf/message.h>
#include <gazebo/msgs/msgs.hh>


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
    AspectProviderAspect("GazeboAspect", &__gazebo_aspect_inifin)
{
}


/** Destructor. */
GazeboNodeThread::~GazeboNodeThread()
{
}


void
GazeboNodeThread::init()
{
  //read config values
  if(config->exists("/gazsim/gazebo-communication-channel"))
  {
    robot_channel = config->get_string("/gazsim/gazebo-communication-channel");
  }
  else
  {
    logger->log_warn(name(), "Please start fawkes with the configuration file for the simulation. The Gazebo node will be initialized with the channel \"\".");
    robot_channel = "";
  }
  if(gazebo::transport::is_stopped()) {
    gazebo::transport::init();
    gazebo::transport::run();
  }
  else {
    logger->log_warn(name(), "Gazebo already running ");
  }

  //Initialize Communication nodes:
  //the common one for the robot
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  __gazebonode = node;
  //initialize node (this node only communicates with nodes that were initialized with the same string)
  __gazebonode->Init(robot_channel.c_str());
  __gazebo_aspect_inifin.set_gazebonode(__gazebonode);
  
  //and the node for world change messages
  __gazebo_world_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  __gazebo_aspect_inifin.set_gazebo_world_node(__gazebo_world_node);
  //with all possible publishers
  __visual_publisher = __gazebo_world_node->Advertise<gazebo::msgs::Visual>("~/visual", 5);
  __model_publisher = __gazebo_world_node->Advertise<gazebo::msgs::Model>("~/model", 5);
  __request_publisher = __gazebo_world_node->Advertise<gazebo::msgs::Request>("~/request", 5);
  __light_publisher = __gazebo_world_node->Advertise<gazebo::msgs::Light>("~/light", 5);
  __gazebo_aspect_inifin.set_visual_publisher(__visual_publisher);
  __gazebo_aspect_inifin.set_model_publisher(__model_publisher);
  __gazebo_aspect_inifin.set_request_publisher(__request_publisher);
  __gazebo_aspect_inifin.set_light_publisher(__light_publisher);
}


void
GazeboNodeThread::finalize()
{
  __gazebonode->Fini();
  __gazebonode.reset();
  __gazebo_aspect_inifin.set_gazebonode(__gazebonode);
  __gazebo_world_node->Fini();
  __gazebo_world_node.reset();
  __gazebo_aspect_inifin.set_gazebonode(__gazebo_world_node);
}


void
GazeboNodeThread::loop()
{
}
