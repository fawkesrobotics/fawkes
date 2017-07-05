/***************************************************************************
 *  robot_memory_inifin.cpp - RobotMemoryAspect initializer/finalizer
 *    
 *
 *  Created: Aug 23, 2016 1:28:04 PM 2016
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
#include "robot_memory_inifin.h"

namespace fawkes
{

/** @class RobotMemoryIniFin robot_memory_inifin.cpp
 * RobotMemoryAspect initializer/finalizer.
 * This initializer/finalizer will provide the RobotMemory to
 * threads with the RobotMemoryAspect.
 * @author Frederik Zwilling
 */

RobotMemoryIniFin::RobotMemoryIniFin()
: AspectIniFin("RobotMemoryAspect")
{}

/** Initialize
 * @param thread thread
 */
void
RobotMemoryIniFin::init(Thread *thread)
{
  RobotMemoryAspect *robot_memory_thread;
  robot_memory_thread = dynamic_cast<RobotMemoryAspect *>(thread);
  if (robot_memory_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
            "RobotMemoryAspect, but RTTI says it "
            "has not. ", thread->name());
  }
  if (! robot_memory_) {
    throw CannotInitializeThreadException("robot_memory object has not been set.");
  }

  robot_memory_thread->init_RobotMemoryAspect(robot_memory_);
}

/** Finilize
 * @param thread thread
 */
void
RobotMemoryIniFin::finalize(Thread *thread)
{
  RobotMemoryAspect *robot_memory_thread;
  robot_memory_thread = dynamic_cast<RobotMemoryAspect *>(thread);
  if (robot_memory_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
            "RobotMemoryAspect, but RTTI says it "
            "has not. ", thread->name());
  }
  robot_memory_thread->finalize_RobotMemoryAspect();
}

/**
 * Set the reference to the robot memory for the aspect
 * @param robot_memory Robot Memory
 */
void
RobotMemoryIniFin::set_robot_memory(RobotMemory* robot_memory)
{
  robot_memory_ = robot_memory;
}

} /* namespace fawkes */
