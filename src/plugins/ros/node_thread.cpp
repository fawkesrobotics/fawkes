
/***************************************************************************
 *  node_thread.cpp - ROS node handle providing Thread
 *
 *  Created: Thu May 05 18:37:08 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include <utils/system/hostinfo.h>

#include <ros/ros.h>

using namespace fawkes;

/** @class ROSNodeThread "node_thread.h"
 * ROS node handle thread.
 * This thread maintains a ROS node which can be used by other
 * threads and is provided via the ROSAspect.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
ROSNodeThread::ROSNodeThread()
  : Thread("ROSNodeThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP),
    AspectProviderAspect(&ros_aspect_inifin_)
{
}


/** Destructor. */
ROSNodeThread::~ROSNodeThread()
{
}


void
ROSNodeThread::init()
{
  cfg_async_spinning_ = false;
  try {
    cfg_async_spinning_ = config->get_bool("/ros/async-spinning");
  } catch (Exception &e) {} // ignored, use default

  cfg_async_num_threads_ = 4;
  try {
    cfg_async_num_threads_ = config->get_uint("/ros/async-num-threads");
  } catch (Exception &e) {} // ignored, use default

  if (! ros::isInitialized()) {
    int argc = 1;
    const char *argv[] = {"fawkes"};
    std::string node_name = "fawkes";
    try {
      node_name = config->get_string("/ros/node-name");
    } catch (Exception &e) {} // ignored, use default
    if (node_name == "$HOSTNAME") {
      HostInfo hinfo;
      node_name = hinfo.short_name();
    }

    ros::init(argc, (char **)argv, node_name,
	      (uint32_t)ros::init_options::NoSigintHandler);
  } else {
    logger->log_warn(name(), "ROS node already initialized");
  }

  if (ros::isStarted()) {
    logger->log_warn(name(), "ROS node already *started*");
  }

  rosnode_ = new ros::NodeHandle();

  ros_aspect_inifin_.set_rosnode(rosnode_);

  if (cfg_async_spinning_) {
    async_spinner_ = new ros::AsyncSpinner(cfg_async_num_threads_);
    async_spinner_->start();
  }
}


void
ROSNodeThread::finalize()
{
  if (cfg_async_spinning_) {
    async_spinner_->stop();
    delete async_spinner_;
  }
  rosnode_->shutdown();

  rosnode_.clear();
  ros_aspect_inifin_.set_rosnode(rosnode_);
}


void
ROSNodeThread::loop()
{
  if (! cfg_async_spinning_) {
    ros::spinOnce();
  }
}
