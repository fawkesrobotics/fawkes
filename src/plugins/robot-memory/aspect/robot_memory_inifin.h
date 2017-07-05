/***************************************************************************
 *  robot_memory_inifin.h - RobotMemoryAspect initializer/finalizer
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
#ifndef FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ASPECT_ROBOT_MEMORY_INIFIN_H_
#define FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ASPECT_ROBOT_MEMORY_INIFIN_H_

#include <aspect/inifins/inifin.h>
#include "robot_memory_aspect.h"

namespace fawkes
{


class RobotMemoryIniFin : public AspectIniFin
{
  public:
    RobotMemoryIniFin();

    virtual void init(Thread *thread);
    virtual void finalize(Thread *thread);

    void set_robot_memory(RobotMemory* robot_memory);

  private:
    RobotMemory* robot_memory_;
};

} /* namespace fawkes */

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ASPECT_ROBOT_MEMORY_INIFIN_H_ */
