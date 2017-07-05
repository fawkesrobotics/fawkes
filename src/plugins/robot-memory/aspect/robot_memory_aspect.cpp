/***************************************************************************
 *  robot_memory_aspect.cpp - Aspect providing the robot memory
 *    
 *
 *  Created: Aug 23, 2016 1:26:08 PM 2016
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
#include "robot_memory_aspect.h"

namespace fawkes
{

/** @class RobotMemoryAspect "robot_memory_aspect.h"
 * Thread aspect to get access to a the RobotMemory.
 *
 * @ingroup Aspects
 * @author Frederik Zwilling
 */

RobotMemoryAspect::RobotMemoryAspect()
{
  add_aspect("RobotMemoryAspect");
}

RobotMemoryAspect::~RobotMemoryAspect()
{}

void
RobotMemoryAspect::init_RobotMemoryAspect(RobotMemory* robot_memory)
{
  this->robot_memory = robot_memory;
}

void
RobotMemoryAspect::finalize_RobotMemoryAspect()
{
  this->robot_memory = NULL;
}

} /* namespace fawkes */
