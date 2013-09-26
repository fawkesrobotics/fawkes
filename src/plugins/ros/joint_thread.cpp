
/***************************************************************************
 *  joint_thread.cpp - Thread to publish JointStates to ROS
 *
 *  Created: Wed Sep 25 18:27:26 2013
 *  Copyright  2013  Till Hofmann
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

#include "joint_thread.h"

#include <core/threading/mutex_locker.h>
#include <ros/this_node.h>
#include <sensor_msgs/JointState.h>

using namespace fawkes;

/** @class RosJointThread "joint_thread.h"
 * Thread to publish JointStates to ROS.
 * This thread reads all Joint Blackboard Interfaces and publishes every
 * joint to ROS.
 * @author Till Hofmann
 */

/** Constructor. */
RosJointThread::RosJointThread()
  : Thread("RosJointThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
    BlackBoardInterfaceListener("RosJointThread")
{
}

/** Destructor. */
RosJointThread::~RosJointThread()
{
}



void
RosJointThread::init()
{
  ros_pub_ = rosnode->advertise<sensor_msgs::JointState>("/joints", 100);
  // check for open JointInterfaces
  ifs_ = blackboard->open_multiple_for_reading<JointInterface>();
  for (std::list<JointInterface *>::iterator it = ifs_.begin(); it != ifs_.end(); it++) {
    bbil_add_data_interface(*it);
  }
  // watch for creation of new JointInterfaces
  bbio_add_observed_create("JointInterface");
  // watch for destruction of JointInterfaces
  bbio_add_observed_destroy("JointInterface");

  // register to blackboard
  blackboard->register_listener(this);
  blackboard->register_observer(this);
}

void
RosJointThread::loop()
{
}


void
RosJointThread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->unregister_observer(this);
  for (std::list<JointInterface *>::iterator it = ifs_.begin(); it != ifs_.end(); it++) {
    blackboard->close(*it);
  }
  ros_pub_.shutdown();
}

void
RosJointThread::bb_interface_created(const char *type, const char *id) throw()
{
  if (strncmp(type, "JointInterface", __INTERFACE_TYPE_SIZE) != 0)  return;
  JointInterface *interface;
  try {
    interface = blackboard->open_for_reading<JointInterface>(id);
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
RosJointThread::bb_interface_destroyed(const char *type, const char *id) throw()
{
  if (strncmp(type, "JointInterface", __INTERFACE_TYPE_SIZE) != 0)  return;
  for (std::list<JointInterface *>::iterator it = ifs_.begin(); it != ifs_.end(); it++) {
    if ((*it)->id() == id) {
      bbil_remove_data_interface(*it);
      blackboard->update_listener(this);
      blackboard->close(*it);
      ifs_.erase(it);
      break;
    }
  }
}

void
RosJointThread::bb_interface_data_changed(Interface *interface) throw()
{
  JointInterface *jiface = dynamic_cast<JointInterface *>(interface);
  if (!jiface) return;
  jiface->read();
  sensor_msgs::JointState joint_state;
//  std::string names[] = { "testJoint" };
  joint_state.name.push_back(jiface->id());
  joint_state.position.push_back(jiface->position());
  joint_state.velocity.push_back(jiface->velocity());
  ros_pub_.publish(joint_state);
}
