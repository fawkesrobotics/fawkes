/***************************************************************************
 *  clips_rm_trigger.h - Class holding information and callback for trigger in CLIPS
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

#ifndef FAWKES_SRC_PLUGINS_CLIPS_ROBOT_MEMORY_CLIPS_RM_TRIGGER_H_
#define FAWKES_SRC_PLUGINS_CLIPS_ROBOT_MEMORY_CLIPS_RM_TRIGGER_H_

#include <string>
#include <plugins/robot-memory/robot_memory.h>
#include <clipsmm.h>
#include <core/utils/lockptr.h>
#include <logging/logger.h>

/** @class ClipsRmTrigger  clips_rm_trigger.h
 *
 * @author Frederik Zwilling
 */
class ClipsRmTrigger
{
  public:
    ClipsRmTrigger(std::string assert_name, RobotMemory *robot_memory, fawkes::LockPtr<CLIPS::Environment> &clips, fawkes::Logger *logger);
    virtual ~ClipsRmTrigger();

    void callback(mongo::BSONObj update);
    void set_trigger(EventTrigger *trigger);

  private:
    std::string assert_name;
    EventTrigger *trigger;
    RobotMemory *robot_memory;
    fawkes::LockPtr<CLIPS::Environment> clips;
    fawkes::Logger *logger;
};

#endif /* FAWKES_SRC_PLUGINS_CLIPS_ROBOT_MEMORY_CLIPS_RM_TRIGGER_H_ */
