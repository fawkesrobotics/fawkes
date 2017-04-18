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
#include <clipsmm.h>

#include <core/threading/mutex_locker.h>

using namespace fawkes;
using namespace mongo;

/**
 * Constructor with references to objects of the plugin
 * @param assert_name String used to identify this trigger in resulting facts
 * @param robot_memory Robot Memory
 * @param clips Clips environment
 * @param logger Logger
 */
ClipsRmTrigger::ClipsRmTrigger(std::string assert_name, RobotMemory *robot_memory,
  LockPtr<CLIPS::Environment> &clips, fawkes::Logger *logger)
{
  this->assert_name = assert_name;
  this->robot_memory = robot_memory;
  this->clips = clips;
  this->logger = logger;
}

ClipsRmTrigger::~ClipsRmTrigger()
{
  if(trigger)
  {
    robot_memory->remove_trigger(trigger);
  }
}

/**
 * Set the trigger object given by the robot memory
 * @param trigger Trigger
 */
void ClipsRmTrigger::set_trigger(EventTrigger *trigger)
{
  this->trigger = trigger;
}

/**
 * Callback function for the trigger. Asserts a fact about the update with the assert_name and updated object.
 * When you retract the fact about the update, also call bson-destroy on the included pointer to avoid memory leaks.
 * @param update updated object
 */
void ClipsRmTrigger::callback(mongo::BSONObj update)
{
  MutexLocker locker(clips.objmutex_ptr());
  clips->assert_fact_f("( %s)", assert_name.c_str());
  CLIPS::Template::pointer temp = clips->get_template("robmem-trigger");
  if (temp) {
    struct timeval tv;
    gettimeofday(&tv, 0);
    CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
    fact->set_slot("name", assert_name.c_str());
    CLIPS::Values rcvd_at(2, CLIPS::Value(CLIPS::TYPE_INTEGER));
    rcvd_at[0] = tv.tv_sec;
    rcvd_at[1] = tv.tv_usec;
    fact->set_slot("rcvd-at", rcvd_at);
    BSONObjBuilder *b = new BSONObjBuilder();
    b->appendElements(update);
    void *ptr = b;
    fact->set_slot("ptr", CLIPS::Value(ptr));
    CLIPS::Fact::pointer new_fact = clips->assert_fact(fact);

    if (!new_fact) {
      logger->log_warn("CLIPS-RobotMemory", "Asserting robmem-trigger fact failed");
      delete static_cast<BSONObjBuilder *>(ptr);
    }
  } else {
    logger->log_warn("CLIPS-RobotMemory",
        "Did not get template, did you load robot-memory.clp?");
  }

}

