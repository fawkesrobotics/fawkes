
/***************************************************************************
 *  stop_drive_mode.cpp - Implementation of drive-mode "stop"
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
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

/** @class CStopDriveModule <plugins/colli/drive_modes/stop_drive_mode.h>
 * Stop-Drive-Module. This module is called, if something goes wrong, or is not recognized.
 * It is a fairly easy one, because it sets all to zero.
 */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
CStopDriveModule::CStopDriveModule(Logger* logger, Configuration* config)
 : CAbstractDriveMode(logger, config)
{
  logger_->log_debug("CStopDriveModule", "(Constructor): Entering...");
  m_DriveModeName = NavigatorInterface::MovingNotAllowed;
  logger_->log_debug("CStopDriveModule", "(Constructor): Exiting...");
}


/** Destruct your local values here. */
CStopDriveModule::~CStopDriveModule()
{
  logger_->log_debug("CStopDriveModule", "(Destructor): Entering...");
  logger_->log_debug("CStopDriveModule", "(Destructor): Exiting...");
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
 *     m_TargetX, m_TargetY, m_TargetOri  --> current Target to drive to
 *     m_RoboX, m_RoboY, m_RoboOri        --> current Robot coordinates
 *     m_RoboTrans, m_RoboRot             --> current Motor values
 *
 *     m_LocalTargetX, m_LocalTargetY     --> our local target found by the search component we want to reach
 *     m_LocalTrajecX, m_LocalTrajecY     --> The point we would collide with, if we would drive WITHOUT Rotation
 *
 *     m_OrientAtTarget                   --> Do we have to orient ourself at the target?
 *     m_StopAtTarget                     --> Do we have to stop really ON the target?
 *
 *  Afterwards filled should be:
 *
 *     m_ProposedTranslation              --> Desired Translation speed
 *     m_ProposedRotation                 --> Desired Rotation speed
 *
 *  Those values are questioned after an Update() was called.
 */
void
CStopDriveModule::Update()
{
  m_ProposedTranslationX = 0.0;
  m_ProposedTranslationY = 0.0;
  m_ProposedRotation    = 0.0;
}

} // namespace fawkes
