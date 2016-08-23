
/***************************************************************************
 *  robot_memory_thread.cpp - Robot Memory thread
 *
 *  Created: Sun May 01 13:41:45 2016
 *  Copyright  2016 Frederik Zwilling
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

#include "robot_memory_thread.h"
#include "interfaces/RobotMemoryInterface.h"
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <memory>


using namespace fawkes;

/** @class RobotMemoryThread "robot_memory_thread.h"
 * Thread that provides a robot memory with MongoDB
 * @author Frederik Zwilling
 */

/** Constructor. */
RobotMemoryThread::RobotMemoryThread()
	: Thread("RobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP),
	  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
    AspectProviderAspect(&robot_memory_inifin_)
{
}


/** Destructor. */
RobotMemoryThread::~RobotMemoryThread()
{}


void
RobotMemoryThread::init()
{
  //init RobotMemory itself
  robot_memory = new RobotMemory(config, logger, clock, mongodb_client, blackboard);
  robot_memory->init();
  //prepare aspect initializer
  robot_memory_inifin_.set_robot_memory(robot_memory);
}


void
RobotMemoryThread::finalize()
{
  robot_memory_inifin_.set_robot_memory(NULL);
  delete robot_memory;
}


void
RobotMemoryThread::loop()
{
	// process interface messages
  while (! robot_memory->rm_if_->msgq_empty() ) {
    if (robot_memory->rm_if_->msgq_first_is<RobotMemoryInterface::QueryMessage>()) {
	    RobotMemoryInterface::QueryMessage* msg = (RobotMemoryInterface::QueryMessage*) robot_memory->rm_if_->msgq_first();
	    robot_memory->exec_query(msg->query());
    } else if (robot_memory->rm_if_->msgq_first_is<RobotMemoryInterface::InsertMessage>()) {
	    RobotMemoryInterface::InsertMessage* msg = (RobotMemoryInterface::InsertMessage*) robot_memory->rm_if_->msgq_first();
	    robot_memory->exec_insert(msg->insert());
    } else if (robot_memory->rm_if_->msgq_first_is<RobotMemoryInterface::UpdateMessage>()) {
	    RobotMemoryInterface::UpdateMessage* msg = (RobotMemoryInterface::UpdateMessage*) robot_memory->rm_if_->msgq_first();
	    robot_memory->exec_update(msg->query(), msg->update());
    } else if (robot_memory->rm_if_->msgq_first_is<RobotMemoryInterface::RemoveMessage>()) {
	    RobotMemoryInterface::RemoveMessage* msg = (RobotMemoryInterface::RemoveMessage*) robot_memory->rm_if_->msgq_first();
	    robot_memory->exec_remove(msg->query());
    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    robot_memory->rm_if_->msgq_pop();
  }
}

