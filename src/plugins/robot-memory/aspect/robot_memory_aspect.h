/***************************************************************************
 *  robot_memory_aspect.h - Aspect providing the robot memory
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
#ifndef FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ASPECT_ROBOT_MEMORY_ASPECT_H_
#define FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ASPECT_ROBOT_MEMORY_ASPECT_H_

#include <aspect/aspect.h>
#include "../robot_memory.h"


namespace fawkes
{

/*
 *
 */
class RobotMemoryAspect : public virtual Aspect
{
  /// Access for RobotMemoryIniFin to set and finalize robot_memory
  friend class RobotMemoryIniFin;

  public:
    RobotMemoryAspect();
    virtual ~RobotMemoryAspect();

  protected:
    /**
     * RobotMemory object for storing and querying information
     */
    RobotMemory* robot_memory;

  private:
   void init_RobotMemoryAspect(RobotMemory* robot_memory);
   void finalize_RobotMemoryAspect();
};

} /* namespace fawkes */

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ASPECT_ROBOT_MEMORY_ASPECT_H_ */
