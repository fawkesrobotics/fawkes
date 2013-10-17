//     A* Collision Avoidance Algorithm by Stefan Jacobs
//     Copyright (C) 2002  Stefan Jacobs <Stefan_J@gmx.de>
//
//     This program is free software; you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation; either version 2 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program; if not, write to the Free Software
//     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//


/*
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
  ©                                                                            ©
  ©                                            ####   ####           .-""-.    ©
  ©       # #                             #   #    # #    #         /[] _ _\   ©
  ©       # #                                 #    # #             _|_o_LII|_  ©
  © ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ ©
  © #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| ©
  © #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  ©
  © #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  ©
  © '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  ©
  ©                                                               /__|    |__\ ©
  ©                                                                            ©
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
*/


/* ******************************************************************** */
/*                                                                      */
/* $Id$ */
/*                                                                      */
/* Description: This is the slow forward and backward drive module      */
/*                implementation of Colli-A*                            */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This is the slow forward and backward drive module.            */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */

#include "slow_biward_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Initialize your local values here.
 */
CSlowBiwardDriveModule::CSlowBiwardDriveModule( CSlowForwardDriveModule*  slow_forward,
                                                CSlowBackwardDriveModule* slow_backward,
                                                Logger* logger,
                                                Configuration* config )
 : CAbstractDriveMode(logger, config)
{
  logger_->log_info("CSlowBiwardDriveModule", "(Constructor): Entering...");
  m_DriveModeName = NavigatorInterface::SlowAllowBackward;
  m_pSlowForwardDriveModule  = slow_forward;
  m_pSlowBackwardDriveModule = slow_backward;

  m_CountForward = 1;

  m_MaxTranslation = config_->get_float( "/plugins/colli/SlowDriveModule/MAX_TRANS" );
  m_MaxRotation    = config_->get_float( "/plugins/colli/SlowDriveModule/MAX_ROT" );

  logger_->log_info("CSlowBiwardDriveModule", "(Constructor): Exiting...");
}


/** Destruct your local values here.
 */
CSlowBiwardDriveModule::~CSlowBiwardDriveModule()
{
  logger_->log_info("CSlowBiwardDriveModule", "(Destructor): Entering...");
  logger_->log_info("CSlowBiwardDriveModule", "(Destructor): Exiting...");
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
CSlowBiwardDriveModule::Update()
{
  // Just to take care.
  m_ProposedTranslation = 0.0;
  m_ProposedRotation    = 0.0;

  // Our drive mode (choose between forward and backward)
  CAbstractDriveMode * driveMode = 0;

  // Search the correct drive mode
  float angle_to_target = atan2( m_LocalTargetY, m_LocalTargetX );

  if ( m_CountForward == 1 && fabs( angle_to_target ) > M_PI_2+0.1 )
    m_CountForward = -1;

  else if ( m_CountForward == 1 )
    m_CountForward = 1;

  else if ( m_CountForward == -1 && fabs( angle_to_target ) < M_PI_2-0.1 )
    m_CountForward = 1;

  else if ( m_CountForward == -1 )
    m_CountForward = -1;

  else {
    logger_->log_debug("CSlowBiwardDriveModule", "Undefined state");
    m_CountForward = 0;
  }

  if ( m_CountForward == 1 )
    driveMode = m_pSlowForwardDriveModule;
  else
    driveMode = m_pSlowBackwardDriveModule;

  // set the current info to the drive mode
  driveMode->SetCurrentRoboPos( m_RoboX, m_RoboY, m_RoboOri );
  driveMode->SetCurrentRoboSpeed( m_RoboTrans, m_RoboRot );
  driveMode->SetCurrentTarget( m_TargetX, m_TargetY, m_TargetOri );
  driveMode->SetLocalTarget( m_LocalTargetX, m_LocalTargetY );
  driveMode->SetLocalTrajec( m_LocalTrajecX, m_LocalTrajecY );
  driveMode->SetCurrentColliMode( m_OrientAtTarget, m_StopAtTarget );

  // update the drive mode
  driveMode->Update();

  // get the values from the drive mode
  m_ProposedTranslation = driveMode->GetProposedTranslation();
  m_ProposedRotation    = driveMode->GetProposedRotation();
}

} // namespace fawkes
