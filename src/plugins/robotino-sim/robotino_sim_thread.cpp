/***************************************************************************
 *  robotino-sim_thread.cpp - Thread simulate the Robotino in Gazebo by sending needed informations to the Robotino-plugin in Gazebo and recieving sensordata from Gazebo
 *
 *  Created: Fr 3. Mai 21:27:06 CEST 2013
 *  Copyright  2013  Frederik Zwilling
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

#include "robotino_sim_thread.h"

#include <tf/types.h>
#include <interfaces/Position3DInterface.h>

using namespace fawkes;

/** @class RobotinoSimThread "robotino-sim_thread.h"
 * Thread simulate the Robotino in Gazebo 
 * by sending needed informations to the Robotino-plugin in Gazebo
 * and recieving sensordata from Gazebo
 * @author Tim Niemueller
 */

/** Constructor. */
RobotinoSimThread::RobotinoSimThread()
  : Thread("RobotinoSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

void
RobotinoSimThread::init()
{
  //pose_if_ = blackboard->open_for_reading<Position3DInterface>("Pose");
}

void
RobotinoSimThread::finalize()
{
  //blackboard->close(pose_if_);
}

void
RobotinoSimThread::loop()

{
  logger->log_info(name(), "Robotino-Sim plugin active");
}
