/***************************************************************************
 *  clips_rm_trigger.cpp - Class holding information and callback for trigger in CLIPS
 *    
 *
 *  Created: 11:57:31 AM 2016
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

#include "clips_rm_trigger.h"

using namespace fawkes;

ClipsRmTrigger::ClipsRmTrigger(std::string assert_name, RobotMemory *robot_memory, LockPtr<CLIPS::Environment> &clips)
{
  this->assert_name = assert_name;
  this->robot_memory = robot_memory;
  this->clips = clips;
}

ClipsRmTrigger::~ClipsRmTrigger()
{
  if(trigger)
  {
    robot_memory->remove_trigger(trigger);
  }
}

void ClipsRmTrigger::set_trigger(EventTrigger *trigger)
{
  this->trigger = trigger;
}

/**
 * Callback function for the trigger. Asserts a fact about the update with the assert_name and updated object.
 * @param update updated object
 */
void ClipsRmTrigger::callback(mongo::BSONObj update)
{
  clips->assert_fact_f("(robmem-update-triggered)");
}

