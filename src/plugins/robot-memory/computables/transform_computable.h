/***************************************************************************
 *  transform_computable.h - Computable for doing transforms
 *    
 *
 *  Created: 4:11:27 PM 2016
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

#ifndef FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_TRANSFORM_COMPUTABLE_H_
#define FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_TRANSFORM_COMPUTABLE_H_

#include "../robot_memory.h"
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <config/config.h>


class TransformComputable
{
  public:
    TransformComputable(RobotMemory* robot_memory, fawkes::tf::Transformer* tf, fawkes::Logger* logger, fawkes::Configuration* config);
    virtual ~TransformComputable();

  private:
    std::list<mongo::BSONObj> compute_transform(mongo::BSONObj query, std::string collection);

    RobotMemory* robot_memory_;
    fawkes::Logger* logger_;
    fawkes::tf::Transformer* tf_;
    const char* name_ = "RobotMemory TransformComputable";
    std::vector<Computable*>  computables;
    fawkes::Configuration* config_;
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_COMPUTABLES_TRANSFORM_COMPUTABLE_H_ */
