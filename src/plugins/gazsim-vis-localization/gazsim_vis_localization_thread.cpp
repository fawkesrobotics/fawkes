/***************************************************************************
 *  gazsim_vis_localization_thread.h - Plugin visualizes the localization
 *
 *  Created: Tue Sep 17 15:38:34 2013
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

#include "gazsim_vis_localization_thread.h"

#include <tf/types.h>
#include <stdio.h>
#include <math.h>
#include <utils/math/angle.h>

#include <interfaces/Position3DInterface.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>

using namespace fawkes;
using namespace gazebo;

/** @class VisLocalizationThread "gazsim_localization_thread.h"
 * Thread simulates the Localization in Gazebo
 * @author Frederik Zwilling
 */

/** Constructor. */
VisLocalizationThread::VisLocalizationThread()
  : Thread("VisLocalizationThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

void VisLocalizationThread::init()
{
  logger->log_debug(name(), "Initializing Visualization of the Localization");

  //open interface
  pose_if_ = blackboard->open_for_reading<Position3DInterface>("Pose");
}

void VisLocalizationThread::finalize()
{
  blackboard->close(pose_if_);
}

void VisLocalizationThread::loop()
{
  //TODO: visulize
}
