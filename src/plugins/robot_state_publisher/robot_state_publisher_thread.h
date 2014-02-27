/***************************************************************************
 *  robot_state_publisher_thread.h - Robot State Publisher Plugin
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


#ifndef __PLUGINS_ROBOTSTATEPUBLISHER_ROBOTSTATEPUBLISHER_THREAD_H_
#define __PLUGINS_ROBOTSTATEPUBLISHER_ROBOTSTATEPUBLISHER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/tf.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>

#include <interfaces/JointInterface.h>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>

#include <map>
#include <list>

/** @class SegmentPair
 * This class represents the segment between a parent and a child joint
 */
class SegmentPair
{
public:
  /** Constructor.
   * @param p_segment The Segment of the joint pair
   * @param p_root The name of the parent joint
   * @param p_tip The name of the child joint
   */
  SegmentPair(const KDL::Segment& p_segment, const std::string& p_root, const std::string& p_tip):
    segment(p_segment), root(p_root), tip(p_tip){}

  /** The segment of the joint pair */
  KDL::Segment segment;
  /** The name of the parent joint */
  std::string root;
  /** The name of the child joint */
  std::string tip;
};

class RobotStatePublisherThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::ClockAspect,
  public fawkes::TransformAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlackBoardInterfaceObserver,
  public fawkes::BlackBoardInterfaceListener
{
public:
  RobotStatePublisherThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  // InterfaceObserver
  virtual void bb_interface_created(const char *type, const char *id) throw();

  // InterfaceListener
  virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();
  virtual void bb_interface_writer_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw();

private:
  void publish_fixed_transforms();

  void add_children(const KDL::SegmentMap::const_iterator segment);
  void transform_kdl_to_tf(const KDL::Frame &k, fawkes::tf::Transform &t);
  bool joint_is_in_model(const char *id);
  void conditional_close(fawkes::Interface *interface) throw();

private:

  std::map<std::string, SegmentPair> segments_, segments_fixed_;
  KDL::Tree tree_;
  std::string cfg_urdf_path_;
  float cfg_postdate_to_future_;

  std::list<fawkes::JointInterface *> ifs_;
};

#endif
