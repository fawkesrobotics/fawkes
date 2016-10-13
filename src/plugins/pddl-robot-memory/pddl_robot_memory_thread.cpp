
/***************************************************************************
 *  pddl_robot_memory_thread.cpp - pddl_robot_memory
 *
 *  Plugin created: Thu Oct 13 13:34:05 2016

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

#include "pddl_robot_memory_thread.h"

using namespace fawkes;

/** @class PddlRobotMemoryThread 'pddl_robot_memory_thread.h' 
 * Generate PDDL files from the robot memory
 * @author Frederik Zwilling
 */

PddlRobotMemoryThread::PddlRobotMemoryThread()
 : Thread("PddlRobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP),
             BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL) 
{
}

void
PddlRobotMemoryThread::init()
{
}

void
PddlRobotMemoryThread::loop()
{
}

void
PddlRobotMemoryThread::finalize()
{
}

