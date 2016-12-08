/***************************************************************************
 *  blackboard_computable.h - Computable providing blackboard access
 *    
 *
 *  Created: 1:22:31 PM 2016
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

#ifndef FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_BLACKBOARD_COMPUTABLE_H_
#define FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_BLACKBOARD_COMPUTABLE_H_

#include "../robot_memory.h"
#include <blackboard/blackboard.h>
#include <aspect/logging.h>
#include <config/config.h>

/** @class BlackboardComputable  blackboard_computable.h
 *
 * @author Frederik Zwilling
 */
class BlackboardComputable
{
  public:
    BlackboardComputable(RobotMemory* robot_memory, fawkes::BlackBoard* blackboard, fawkes::Logger* logger, fawkes::Configuration* config);
    virtual ~BlackboardComputable();

  private:
    std::list<mongo::BSONObj> compute_interfaces(mongo::BSONObj query, std::string collection);

    RobotMemory* robot_memory_;
    fawkes::BlackBoard* blackboard_;
    fawkes::Logger* logger_;
    const char* name_ = "RobotMemory BlackoardComputable";
    Computable* computable;
    mongo::BSONArray get_interface_fields(fawkes::InterfaceFieldIterator it);
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_BLACKBOARD_COMPUTABLE_H_ */
