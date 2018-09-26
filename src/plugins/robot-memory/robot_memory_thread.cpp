
/***************************************************************************
 *  robot_memory_thread.cpp - Robot Memory thread
 *
 *  Created: Sun May 01 13:41:45 2016
 *  Copyright  2016 Frederik Zwilling
 *             2017 Tim Niemueller [www.niemueller.de]
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
#include <chrono>

using namespace fawkes;

/** @class RobotMemoryThread "robot_memory_thread.h"
 * Thread that provides a robot memory with MongoDB
 * @author Frederik Zwilling
 */

/** Constructor for thread */
RobotMemoryThread::RobotMemoryThread()
	: Thread("RobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
    AspectProviderAspect(&robot_memory_inifin_)
{
}


/** Destructor. */
RobotMemoryThread::~RobotMemoryThread()
{
}

void
RobotMemoryThread::init()
{
  //init RobotMemory itself
  robot_memory = new RobotMemory(config, logger, clock, mongodb_connmgr, blackboard);
  robot_memory->init();
  //prepare aspect initializer
  robot_memory_inifin_.set_robot_memory(robot_memory);

  //register computables
  blackboard_computable = new BlackboardComputable(robot_memory, blackboard, logger, config);
  transform_computable = new TransformComputable(robot_memory, tf_listener, logger, config);
}


void
RobotMemoryThread::finalize()
{
  delete blackboard_computable;
  delete transform_computable;
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
	    QResCursor res = robot_memory->query(msg->query(), msg->collection());
	    //output result
	    std::string query = msg->query();
	    std::string result = "Result of query " + query + ":\n";
	    while(res->more())
	    {
	      mongo::BSONObj doc = res->next();
	      result += doc.toString() + "\n";
	    }
	    logger->log_info(name(), "%s", result.c_str());
	    robot_memory->rm_if_->set_result(result.c_str());
    } else if (robot_memory->rm_if_->msgq_first_is<RobotMemoryInterface::InsertMessage>()) {
	    RobotMemoryInterface::InsertMessage* msg = (RobotMemoryInterface::InsertMessage*) robot_memory->rm_if_->msgq_first();
	    robot_memory->insert(msg->insert()), msg->collection();
    } else if (robot_memory->rm_if_->msgq_first_is<RobotMemoryInterface::UpdateMessage>()) {
	    RobotMemoryInterface::UpdateMessage* msg = (RobotMemoryInterface::UpdateMessage*) robot_memory->rm_if_->msgq_first();
	    robot_memory->update(msg->query(), msg->update(), msg->collection());
    } else if (robot_memory->rm_if_->msgq_first_is<RobotMemoryInterface::RemoveMessage>()) {
	    RobotMemoryInterface::RemoveMessage* msg = (RobotMemoryInterface::RemoveMessage*) robot_memory->rm_if_->msgq_first();
	    robot_memory->remove(msg->query(), msg->collection());
    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    robot_memory->rm_if_->msgq_pop();
  }

  robot_memory->loop();
}

