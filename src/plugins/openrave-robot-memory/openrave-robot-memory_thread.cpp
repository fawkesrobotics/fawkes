
/***************************************************************************
 *  openrave-robot-memory_thread.cpp - openrave-robot-memory
 *
 *  Created: Thu Nov 24 13:14:33 2016
 *  Copyright  2016  Frederik Zwilling
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

#include "openrave-robot-memory_thread.h"

using namespace fawkes;

/** @class OpenraveRobotMemoryThread 'openrave-robot-memory_thread.h' 
 * Creates an OpenRave Scene for motion planning from data in the robot memory
 * @author Frederik Zwilling
 */

/** Constructor. */
OpenraveRobotMemoryThread::OpenraveRobotMemoryThread()
 : Thread("OpenraveRobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP),
   BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT) 
{
}

void
OpenraveRobotMemoryThread::init()
{
  collection_ = config->get_string("plugins/openrave-robot-memory/input-collection");
  openrave_if_ = blackboard->open_for_reading<OpenRaveInterface>(config->get_string("plugins/openrave-robot-memory/openrave-if-name").c_str());
  or_rm_if_ = blackboard->open_for_writing<OpenraveRobotMemoryInterface>(config->get_string("plugins/openrave-robot-memory/if-name").c_str());
}

void
OpenraveRobotMemoryThread::loop()
{
  // process interface messages
  while (! or_rm_if_->msgq_empty() ) {
    if (or_rm_if_->msgq_first_is<OpenraveRobotMemoryInterface::ConstructSceneMessage>()) {
      construct_scene();
    } else {
      logger->log_warn(name(), "Unknown message received");
    }
    or_rm_if_->msgq_pop();
  }
}

void
OpenraveRobotMemoryThread::finalize()
{
}

void
OpenraveRobotMemoryThread::construct_scene()
{
  logger->log_info(name(), "Constructing Scene");
}
