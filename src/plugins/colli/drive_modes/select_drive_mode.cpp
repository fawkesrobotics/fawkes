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
/* $Id$   */
/*                                                                      */
/* Description: This is an abstract drive module interface of Colli-A*  */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This is the select-drive-mode module.                          */
/*       Call class for all other drive modes.                          */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#include "select_drive_mode.h"
#include "abstract_drive_mode.h"

// INCLUDE HERE YOUR DRIVE MODES!!!
#include "stop_drive_mode.h"
#include "escape_drive_mode.h"
#include "slow_forward_drive_mode.h"
#include "slow_backward_drive_mode.h"
#include "slow_biward_drive_mode.h"
#include "medium_forward_drive_mode.h"
#include "medium_backward_drive_mode.h"
#include "medium_biward_drive_mode.h"
#include "fast_forward_drive_mode.h"
#include "fast_backward_drive_mode.h"
#include "fast_biward_drive_mode.h"
// YOUR CHANGES SHOULD END HERE!!!

#include "../utils/rob/robo_motorcontrol.h"
#include "../utils/rob/robo_laser.h"

#include <interfaces/NavigatorInterface.h>
#include <logging/logger.h>
#include <config/config.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

//~ using namespace std;


CSelectDriveMode::CSelectDriveMode( MotorControl* motor,
                                    Laser* laser,
                                    NavigatorInterface* target,
                                    Logger* logger,
                                    Configuration* config )
 : logger_( logger ),
   config_( config )
{
  logger_->log_info("CSelectDriveMode", "(Constructor): Entering");
  m_EscapeFlag   = 0;       // no escaping at the beginning
  m_pMotor       = motor;
  m_pLaser       = laser;
  m_pColliTarget = target;
  m_vDriveModeList.clear();

  logger_->log_debug("CSelectDriveMode", "Creating Drive Mode Objects");


  // ============================
  // APPEND YOUR DRIVE MODE HERE!

  // MISC MODES
  // stop drive mode
  m_vDriveModeList.push_back( (CAbstractDriveMode *)new CStopDriveModule(logger, config) );

  // and here an example of using extra data, e.g. the laser for escape...
  // escape drive mode
  m_vDriveModeList.push_back( (CAbstractDriveMode *)new CEscapeDriveModule(laser, logger, config) );




  // SLOW MODES
  // slow forward drive mode (have to remember for biward driving!
  CSlowForwardDriveModule* slow_forward = new CSlowForwardDriveModule(logger, config);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) slow_forward );

  // slow backward drive mode (have to remember for biward driving!
  CSlowBackwardDriveModule* slow_backward = new CSlowBackwardDriveModule(logger, config);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) slow_backward );

  // slow biward drive mode (takes both forward and backward drive modes as argument!
  m_vDriveModeList.push_back( (CAbstractDriveMode *) new CSlowBiwardDriveModule(slow_forward,
                                                                                slow_backward,
                                                                                logger,
                                                                                config) );

  // MEDIUM MODES
  // medium forward drive mode (have to remember for biward driving!
  CMediumForwardDriveModule* medium_forward = new CMediumForwardDriveModule(logger, config);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) medium_forward );

  // medium backward drive mode (have to remember for biward driving!
  CMediumBackwardDriveModule* medium_backward = new CMediumBackwardDriveModule(logger, config);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) medium_backward );

  // medium biward drive mode (takes both forward and backward drive modes as argument!
  m_vDriveModeList.push_back( (CAbstractDriveMode *) new CMediumBiwardDriveModule(medium_forward,
                                                                                  medium_backward,
                                                                                  logger,
                                                                                  config) );

  // FAST MODES
  // fast forward drive mode (have to remember for biward driving!
  CFastForwardDriveModule* fast_forward = new CFastForwardDriveModule(logger, config);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) fast_forward );

  // fast backward drive mode (have to remember for biward driving!
  CFastBackwardDriveModule* fast_backward = new CFastBackwardDriveModule(logger, config);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) fast_backward );

  // fast biward drive mode (takes both forward and backward drive modes as argument!
  m_vDriveModeList.push_back( (CAbstractDriveMode *) new CFastBiwardDriveModule(fast_forward,
                                                                                fast_backward,
                                                                                logger,
                                                                                config) );

  // YOUR CHANGES SHOULD END HERE!
  // =============================

  logger_->log_info("CSelectDriveMode", "(Constructor): Exiting");
}


CSelectDriveMode::~CSelectDriveMode()
{
  logger_->log_info("CSelectDriveMode", "(Destructor): Entering");
  for ( unsigned int i = 0; i < m_vDriveModeList.size(); i++ )
    delete m_vDriveModeList[i];
  logger_->log_info("CSelectDriveMode", "(Destructor): Exiting");
}


void
CSelectDriveMode::SetLocalTarget( float localTargetX, float localTargetY )
{
  m_LocalTargetX = localTargetX;
  m_LocalTargetY = localTargetY;
}


void
CSelectDriveMode::SetLocalTrajec( float localTrajecX, float localTrajecY )
{
  m_LocalTrajecX = localTrajecX;
  m_LocalTrajecY = localTrajecY;
}


float
CSelectDriveMode::GetProposedTranslation()
{
  return m_ProposedTranslation;
}


float
CSelectDriveMode::GetProposedRotation()
{
  return m_ProposedRotation;
}


/* ****************************************************************************** */
/* ****************************************************************************** */
/*                               U P D A T E                                      */
/* ****************************************************************************** */
/* ****************************************************************************** */

void
CSelectDriveMode::Update( bool escape )
{
  CAbstractDriveMode * m_pDriveMode = 0;
  m_ProposedTranslation = 0.0;
  m_ProposedRotation = 0.0;

  // choose the correct drive mode!
  NavigatorInterface::DriveMode desiredMode = NavigatorInterface::MovingNotAllowed;
  if ( escape == true ) {
    if( m_EscapeFlag == 0
     && m_pMotor->GetMotorDesiredTranslation() != 0
     && m_pMotor->GetMotorDesiredRotation() != 0 ) {
      desiredMode = NavigatorInterface::MovingNotAllowed;
      // we have not yet stopped!

    } else {
      // we have stopped recently, so do escape!
      m_EscapeFlag = 1;
      desiredMode = NavigatorInterface::ESCAPE;
    }

  } else {
    m_EscapeFlag = 0;
    desiredMode  = m_pColliTarget->drive_mode();
  }

  // now search this specific drive mode in the list
  for ( unsigned int i = 0; i < m_vDriveModeList.size(); i++ ) {
    // error checking
    if ( m_vDriveModeList[i]->GetDriveModeName() == desiredMode
     &&  m_pDriveMode != 0 ) {
      logger_->log_error("CSelectDriveMode", "Error while selecting drive mode. There is more than one mode with the same name!!! Stopping!");
      m_pDriveMode = 0;
      break;
    }

    // drive mode checking
    if ( m_vDriveModeList[i]->GetDriveModeName() == desiredMode
     &&  m_pDriveMode == 0 ) {
      m_pDriveMode = m_vDriveModeList[i];
    }
  }


  if ( m_pDriveMode == 0 ) {
    // invalid pointer
    logger_->log_error("CSelectDriveMode", "INVALID DRIVE MODE POINTER, stopping!");
    m_ProposedTranslation = 0.0;
    m_ProposedRotation = 0.0;

  } else {
    // valid drive mode!
    // set the values for the drive mode
    m_pDriveMode->SetCurrentRoboPos( m_pMotor->GetCurrentX(),
                                     m_pMotor->GetCurrentY(),
                                     m_pMotor->GetCurrentOri() );

    m_pDriveMode->SetCurrentRoboSpeed( m_pMotor->GetMotorDesiredTranslation(),
                                       m_pMotor->GetMotorDesiredRotation() );

    m_pDriveMode->SetCurrentTarget( m_pColliTarget->dest_x(),
                                    m_pColliTarget->dest_y(),
                                    m_pColliTarget->dest_ori() );

    m_pDriveMode->SetLocalTarget( m_LocalTargetX, m_LocalTargetY );
    m_pDriveMode->SetLocalTrajec( m_LocalTrajecX, m_LocalTrajecY );
    m_pDriveMode->SetCurrentColliMode( m_pColliTarget->is_orient_at_target(), m_pColliTarget->is_stop_at_target() );

    // update the drive mode
    m_pDriveMode->Update();

    // get the values from the drive mode
    m_ProposedTranslation = m_pDriveMode->GetProposedTranslation();
    m_ProposedRotation    = m_pDriveMode->GetProposedRotation();

    // recheck with targetobj maximum settings
    if( (m_pColliTarget->max_velocity() != 0.0)
     && (fabs( m_ProposedTranslation ) > fabs( m_pColliTarget->max_velocity() )) ) {
      if ( m_ProposedTranslation > 0.0 )
        m_ProposedTranslation = m_pColliTarget->max_velocity();
      else
        m_ProposedTranslation = -m_pColliTarget->max_velocity();
    }

    if( ( m_pColliTarget->max_rotation() != 0.0 )
     && (fabs( m_ProposedRotation ) > fabs( m_pColliTarget->max_rotation() )) ) {
      if ( m_ProposedRotation > 0.0 )
        m_ProposedRotation = m_pColliTarget->max_rotation();
      else
        m_ProposedRotation = -m_pColliTarget->max_rotation();
    }

  }
}

} // namespace fawkes