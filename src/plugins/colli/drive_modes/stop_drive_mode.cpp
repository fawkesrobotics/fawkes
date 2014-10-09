
/***************************************************************************
 *  stop_drive_mode.cpp - Implementation of drive-mode "stop"
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013-2014  Bahram Maleki-Fard
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

#include "stop_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class StopDriveModule <plugins/colli/drive_modes/stop_drive_mode.h>
 * Stop-Drive-Module. This module is called, if something goes wrong, or is not recognized.
 * It is a fairly easy one, because it sets all to zero.
 */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
StopDriveModule::StopDriveModule(Logger* logger, Configuration* config)
 : AbstractDriveMode(logger, config)
{
  logger_->log_debug("StopDriveModule", "(Constructor): Entering...");
  drive_mode_ = NavigatorInterface::MovingNotAllowed;
  logger_->log_debug("StopDriveModule", "(Constructor): Exiting...");
}


/** Destruct your local values here. */
StopDriveModule::~StopDriveModule()
{
  logger_->log_debug("StopDriveModule", "(Destructor): Entering...");
  logger_->log_debug("StopDriveModule", "(Destructor): Exiting...");
}


/* ************************************************************************** */
/* ***********************        U P D A T E       ************************* */
/* ************************************************************************** */

/** Calculate here your desired settings. What you desire is checked afterwards to the current
 *    settings of the physical boundaries, but take care also.
 *
 *  How you do this is up to you, but be careful, our hardware is expensive!!!!
 *
 *  Available are:
 *
 *     target_     --> current target coordinates to drive to
 *     robot_      --> current robot coordinates
 *     robot_vel_  --> current Motor velocities
 *
 *     local_target_      --> our local target found by the search component we want to reach
 *     local_trajec_      --> The point we would collide with, if we would drive WITHOUT Rotation
 *
 *     orient_at_target_  --> Do we have to orient ourself at the target?
 *     stop_at_target_    --> Do we have to stop really ON the target?
 *
 *  Afterwards filled should be:
 *
 *     proposed_          --> Desired translation and rotation speed
 *
 *  Those values are questioned after an update() was called.
 */
void
StopDriveModule::update()
{
  proposed_.x = proposed_.y = proposed_.rot = 0.f;
}

} // namespace fawkes
