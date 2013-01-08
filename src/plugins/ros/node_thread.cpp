
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
    AspectProviderAspect("ROSAspect", &__ros_aspect_inifin)
{
}


/** Destructor. */
ROSNodeThread::~ROSNodeThread()
{
}


void
ROSNodeThread::init()
{
  if (! ros::isInitialized()) {
    int argc = 1;
    const char *argv[] = {"fawkes"};
    std::string node_name = "fawkes";
    try {
      node_name = config->get_string("/ros/node-name");
    } catch (Exception &e) {} // ignored, use default
    ros::init(argc, (char **)argv, node_name,
	      (uint32_t)ros::init_options::NoSigintHandler);
  } else {
    logger->log_warn(name(), "ROS node already initialized");
  }

  if (ros::isStarted()) {
    logger->log_warn(name(), "ROS node already *started*");
  }

  __rosnode = new ros::NodeHandle();

  __ros_aspect_inifin.set_rosnode(__rosnode);
}


void
ROSNodeThread::finalize()
{
  __rosnode->shutdown();

  __rosnode.clear();
  __ros_aspect_inifin.set_rosnode(__rosnode);
}


void
ROSNodeThread::loop()
{
  ros::spinOnce();
}
