
/***************************************************************************
 *  robot_memory_test_thread.cpp - robot_memory_test
 *
 *  Plugin created: Wed Aug 24 13:37:27 2016

 *  Copyright  2016  Frederik Zwilling
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

#include "robot_memory_test_thread.h"
#include <core/exception.h>
#include <baseapp/run.h>
#include <gtest/gtest.h>

using namespace fawkes;

/** @class RobotMemoryTestThread 'robot_memory_test_thread.h' 
 * gtests for the RobotMemory
 * @author Frederik Zwilling
 */

RobotMemoryTestThread::RobotMemoryTestThread()
 : Thread("RobotMemoryTestThread", Thread::OPMODE_WAITFORWAKEUP),
             BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL) 
{
}

void
RobotMemoryTestThread::init()
{
  //prepare tests
  logger->log_warn(name(), "Preparing tests");
  test_env_ = new RobotMemoryTestEnvironment(robot_memory, blackboard);
  ::testing::AddGlobalTestEnvironment((testing::Environment*) test_env_);

}

void
RobotMemoryTestThread::loop()
{
  logger->log_warn(name(), "Starting tests");
  test_result_ = RUN_ALL_TESTS();
  usleep(100000);
  logger->log_warn(name(), "Finished tests with, quitting as intended...");
  //stop fawkes to finish the testing run
  fawkes::runtime::quit();
}

void
RobotMemoryTestThread::finalize()
{
}

