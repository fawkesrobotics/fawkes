/***************************************************************************
 *  robot_state_publisher_thread.cpp - Robot State Publisher Plugin
 *
 *  Created on Thu Aug 22 11:19:00 2013
 *  Copyright (C) 2013 by Till Hofmann, AllemaniACs RoboCup Team
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

/* This code is based on ROS robot_state_publisher and ROS geometry
 * with the following copyright and license:
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include "robot_state_publisher_thread.h"
#include <kdl/frames_io.hpp>
#include<kdl_parser/kdl_parser.hpp>

#include <fstream>
#include <list>

#define CFG_PREFIX "/robot_state_publisher/"

using namespace fawkes;
using namespace std;

/** @class RobotStatePublisherThread "robot_state_publisher_thread.h"
 * Thread to publish the robot's transforms
 * @author Till Hofmann
 */

/** Constructor. */
RobotStatePublisherThread::RobotStatePublisherThread()
: Thread("RobotStatePublisherThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
  TransformAspect(TransformAspect::ONLY_PUBLISHER, "robot_state_transforms"),
  BlackBoardInterfaceListener("RobotStatePublisher")
{
}

void RobotStatePublisherThread::init()
{
  cfg_urdf_path_ = config->get_string(CFG_PREFIX"urdf_file");

  string urdf;
  string line;
  ifstream urdf_file(cfg_urdf_path_);
  if (!urdf_file.is_open()) {
    logger->log_error(name(), "failed to open URDF File %s", cfg_urdf_path_.c_str());
    throw;
  }
  while ( getline(urdf_file, line)) {
    urdf += line;
  }
  urdf_file.close();

  if (!kdl_parser::treeFromString(urdf, tree_)) {
    logger->log_error(name(), "failed to parse urdf description to tree");
    throw;
  }
  // walk the tree and add segments to segments_
  add_children(tree_.getRootSegment());

  // publish robot model to ROS
  ros::param::set("/robot_description", urdf);

  std::map<std::string, SegmentPair> unknown_segments = segments_;

  // check for open JointInterfaces
  std::list<fawkes::JointInterface *> ifs = blackboard->open_multiple_for_reading<JointInterface>();
  for (std::list<JointInterface *>::iterator it = ifs.begin(); it != ifs.end(); it++) {
    if (joint_is_in_model((*it)->id())) {
      logger->log_debug(name(), "Found joint information for %s", (*it)->id());
      unknown_segments.erase((*it)->id());
      ifs_.push_back(*it);
      bbil_add_data_interface(*it);
    }
    else {
      blackboard->close(*it);
    }
  }
  for (map<string, SegmentPair>::const_iterator it = unknown_segments.begin();
      it != unknown_segments.end(); it++) {
    logger->log_warn(name(), "No information for joint %s available", it->first.c_str());
  }
  // watch for creation of new JointInterfaces
  bbio_add_observed_create("JointInterface");
  // watch for destruction of JointInterfaces
  bbio_add_observed_destroy("JointInterface");

  // register to blackboard
  blackboard->register_listener(this);
  blackboard->register_observer(this);
}

void RobotStatePublisherThread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->unregister_observer(this);
  for (std::list<JointInterface *>::iterator it = ifs_.begin(); it != ifs_.end(); it++) {
    blackboard->close(*it);
  }
}

void RobotStatePublisherThread::loop()
{
  publish_fixed_transforms();
}


// add children to correct maps
void RobotStatePublisherThread::add_children(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = segment->second.segment.getName();

  const std::vector<KDL::SegmentMap::const_iterator>& children = segment->second.children;
  for (unsigned int i=0; i<children.size(); i++){
    const KDL::Segment& child = children[i]->second.segment;
    SegmentPair s(children[i]->second.segment, root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None){
      segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
      logger->log_debug(name(), "Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    else{
      segments_.insert(make_pair(child.getJoint().getName(), s));
      logger->log_debug(name(), "Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    add_children(children[i]);
  }
}

// publish fixed transforms
void RobotStatePublisherThread::publish_fixed_transforms()
{
  std::vector<tf::StampedTransform> tf_transforms;
  tf::StampedTransform tf_transform;
  fawkes::Time now(clock);
  tf_transform.stamp = now + 0.5;  // future publish by 0.5 seconds

  // loop over all fixed segments
  for (map<string, SegmentPair>::const_iterator seg=segments_fixed_.begin(); seg != segments_fixed_.end(); seg++){
    transform_kdl_to_tf(seg->second.segment.pose(0), tf_transform);
    tf_transform.frame_id = seg->second.root;
    tf_transform.child_frame_id = seg->second.tip;
    tf_transforms.push_back(tf_transform);
  }
  for (std::vector<tf::StampedTransform>::const_iterator it = tf_transforms.begin();
      it != tf_transforms.end(); it++) {
    tf_publisher->send_transform(*it);
  }
}

void RobotStatePublisherThread::transform_kdl_to_tf(const KDL::Frame &k, fawkes::tf::Transform &t)
  {
    t.setOrigin(tf::Vector3(k.p[0], k.p[1], k.p[2]));
    t.setBasis(tf::Matrix3x3(k.M.data[0], k.M.data[1], k.M.data[2],
                           k.M.data[3], k.M.data[4], k.M.data[5],
                           k.M.data[6], k.M.data[7], k.M.data[8]));
  }


/**
 * @return true if the joint (represented by the interface) is part of our robot model
 */
bool RobotStatePublisherThread::joint_is_in_model(const char *id) {
  return (segments_.find(id) != segments_.end());
}

// InterfaceObserver
void
RobotStatePublisherThread::bb_interface_created(const char *type, const char *id) throw()
{
  if (strncmp(type, "JointInterface", __INTERFACE_TYPE_SIZE) != 0)  return;
  if (!joint_is_in_model(id)) return;
  JointInterface *interface;
  try {
    interface = blackboard->open_for_reading<JointInterface>(id);
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
    return;
  }
  logger->log_debug(name(), "Found joint information for %s", interface->id());
  try {
    ifs_.push_back(interface);
    bbil_add_data_interface(interface);
    blackboard->update_listener(this);
  } catch (Exception &e) {
    blackboard->close(interface);
    logger->log_warn(name(), "Failed to register for %s:%s: %s", type, id, e.what());
    return;
  }
}

void
RobotStatePublisherThread::bb_interface_destroyed(const char *type, const char *id) throw()
{
  if (strncmp(type, "JointInterface", __INTERFACE_TYPE_SIZE) != 0)  return;
  for (std::list<JointInterface *>::iterator it = ifs_.begin(); it != ifs_.end(); it++) {
    if ((*it)->id() == id) {
      logger->log_warn(name(), "JointInterface %s removed, but part of robot model", id);
      bbil_remove_data_interface(*it);
      blackboard->update_listener(this);
      blackboard->close(*it);
      ifs_.erase(it);
      break;
    }
  }
}

// InterfaceListener
void
RobotStatePublisherThread::bb_interface_data_changed(fawkes::Interface *interface) throw()
{
  JointInterface *jiface = dynamic_cast<JointInterface *>(interface);
  if (!jiface) return;
  jiface->read();
  std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(jiface->id());
  if (seg == segments_.end()) return;
  tf::StampedTransform transform;
  transform.stamp = fawkes::Time(clock);
  transform.frame_id = seg->second.root;
  transform.child_frame_id = seg->second.tip;
  transform_kdl_to_tf(seg->second.segment.pose(jiface->position()), transform);
  tf_publisher->send_transform(transform);

}
