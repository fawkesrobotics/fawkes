
/***************************************************************************
 *  select_drive_mode.cpp - Class that selects drive-mode from a collection
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
 *             2014  Tobias Neumann
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

#include "../common/types.h"
#include "select_drive_mode.h"
#include "abstract_drive_mode.h"

// INCLUDE HERE YOUR DRIVE MODES!!!
#include "stop_drive_mode.h"
#include "escape_drive_mode.h"
#include "escape_potential_field_drive_mode.h"
#include "slow_forward_drive_mode.h"
#include "slow_backward_drive_mode.h"
#include "slow_biward_drive_mode.h"
#include "medium_forward_drive_mode.h"
#include "medium_backward_drive_mode.h"
#include "medium_biward_drive_mode.h"
#include "fast_forward_drive_mode.h"
#include "fast_backward_drive_mode.h"
#include "fast_biward_drive_mode.h"

#include "slow_forward_drive_mode_omni.h"
#include "escape_potential_field_drive_mode_omni.h"
// YOUR CHANGES SHOULD END HERE!!!

#include "../utils/rob/robo_motorcontrol.h"
#include "../search/og_laser.h"

#include <interfaces/NavigatorInterface.h>
#include <logging/logger.h>
#include <config/config.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CSelectDriveMode <plugins/colli/drive_modes/select_drive_mode.h>
 * This class selects the correct drive mode and calls the appopriate drive component
 */

/** Constructor.
 * @param motor       The motor controller object
 * @param target      The "colli target" NavigatorInterface
 * @param logger      The fawkes logger
 * @param config      The fawkes configuration
 * @param escape_mode The chosen escape mode
 */
CSelectDriveMode::CSelectDriveMode( MotorControl* motor,
                                    NavigatorInterface* target,
                                    Logger* logger,
                                    Configuration* config,
                                    fawkes::colli_escape_mode_t escape_mode )
 : logger_( logger ),
   config_( config ),
   cfg_escape_mode( escape_mode )
{
  logger_->log_debug("CSelectDriveMode", "(Constructor): Entering");
  m_EscapeFlag   = 0;       // no escaping at the beginning
  m_pMotor       = motor;
  m_pColliTarget = target;
  m_vDriveModeList.clear();

  std::string drive_restriction = config->get_string("/plugins/colli/drive_mode/restriction");

  if (        drive_restriction.compare("omnidirectional") == 0 ) {
    drive_restriction_ = fawkes::colli_drive_restriction_t::omnidirectional;
  } else if ( drive_restriction.compare("differential") == 0 ) {
    drive_restriction_ = fawkes::colli_drive_restriction_t::differential;
  } else {
    drive_restriction_ = fawkes::colli_drive_restriction_t::differential;
    throw fawkes::Exception("Drive restriction is unknown, use differential");
  }

  logger_->log_debug("CSelectDriveMode", "Creating Drive Mode Objects");

  // Add generic drive modes
  m_vDriveModeList.push_back( (CAbstractDriveMode *)new CStopDriveModule(logger_, config_) );

  // Add specific drive modes
  if (        drive_restriction_ == fawkes::colli_drive_restriction_t::omnidirectional ) {
    addDriveModesOmnidirectional();
  } else if ( drive_restriction_ == fawkes::colli_drive_restriction_t::differential ) {
    addDriveModesDifferential();
  } else {
    throw fawkes::Exception("Unknown drive restriction, needs to be implemented");
  }

  logger_->log_debug("CSelectDriveMode", "(Constructor): Exiting");
}

CSelectDriveMode::~CSelectDriveMode()
{
  logger_->log_debug("CSelectDriveMode", "(Destructor): Entering");
  for ( unsigned int i = 0; i < m_vDriveModeList.size(); i++ )
    delete m_vDriveModeList[i];
  logger_->log_debug("CSelectDriveMode", "(Destructor): Exiting");
}

void
CSelectDriveMode::addDriveModesDifferential()
{
  // escape drive mode
  if (cfg_escape_mode == fawkes::colli_escape_mode_t::potential_field) {
    m_vDriveModeList.push_back( (CAbstractDriveMode *)new CEscapePotentialFieldDriveModule( logger_, config_) );
  } else if (cfg_escape_mode == fawkes::colli_escape_mode_t::basic) {
    m_vDriveModeList.push_back( (CAbstractDriveMode *)new CEscapeDriveModule( logger_, config_) );
  } else {
    logger_->log_error("CSelectDriveMode", "Unknown escape drive mode. Using basic as default");
    m_vDriveModeList.push_back( (CAbstractDriveMode *)new CEscapeDriveModule( logger_, config_) );
  }

  // SLOW MODES
  // slow forward drive mode (have to remember for biward driving!
  CSlowForwardDriveModule* slow_forward = new CSlowForwardDriveModule(logger_, config_);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) slow_forward );

  // slow backward drive mode (have to remember for biward driving!
  CSlowBackwardDriveModule* slow_backward = new CSlowBackwardDriveModule(logger_, config_);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) slow_backward );

  // slow biward drive mode (takes both forward and backward drive modes as argument!
  m_vDriveModeList.push_back( (CAbstractDriveMode *) new CSlowBiwardDriveModule(slow_forward,
                                                                                slow_backward,
                                                                                logger_,
                                                                                config_) );

  // MEDIUM MODES
  // medium forward drive mode (have to remember for biward driving!
  CMediumForwardDriveModule* medium_forward = new CMediumForwardDriveModule(logger_, config_);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) medium_forward );

  // medium backward drive mode (have to remember for biward driving!
  CMediumBackwardDriveModule* medium_backward = new CMediumBackwardDriveModule(logger_, config_);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) medium_backward );

  // medium biward drive mode (takes both forward and backward drive modes as argument!
  m_vDriveModeList.push_back( (CAbstractDriveMode *) new CMediumBiwardDriveModule(medium_forward,
                                                                                  medium_backward,
                                                                                  logger_,
                                                                                  config_) );

  // FAST MODES
  // fast forward drive mode (have to remember for biward driving!
  CFastForwardDriveModule* fast_forward = new CFastForwardDriveModule(logger_, config_);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) fast_forward );

  // fast backward drive mode (have to remember for biward driving!
  CFastBackwardDriveModule* fast_backward = new CFastBackwardDriveModule(logger_, config_);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) fast_backward );

  // fast biward drive mode (takes both forward and backward drive modes as argument!
  m_vDriveModeList.push_back( (CAbstractDriveMode *) new CFastBiwardDriveModule(fast_forward,
                                                                                fast_backward,
                                                                                logger_,
                                                                                config_) );
}


void
CSelectDriveMode::addDriveModesOmnidirectional()
{
  // escape drive mode
  if (cfg_escape_mode == fawkes::colli_escape_mode_t::potential_field) {
    m_vDriveModeList.push_back( (CAbstractDriveMode *)new CEscapePotentialFieldOmniDriveModule( logger_, config_) );
  } else if (cfg_escape_mode == fawkes::colli_escape_mode_t::basic) {
    m_vDriveModeList.push_back( (CAbstractDriveMode *)new CEscapeDriveModule( logger_, config_) );                    // This is an differential drive mode
  } else {
    logger_->log_error("CSelectDriveMode", "Unknown escape drive mode. Using potential field omni as default");
    m_vDriveModeList.push_back( (CAbstractDriveMode *)new CEscapePotentialFieldOmniDriveModule( logger_, config_) );
  }

  CSlowForwardOmniDriveModule* slow_forward = new CSlowForwardOmniDriveModule(logger_, config_);
  m_vDriveModeList.push_back( (CAbstractDriveMode *) slow_forward );
}

/** Set local target point before update!
 * @param localTargetX x-coordinate
 * @param localTargetY y-coordinate
 */
void
CSelectDriveMode::SetLocalTarget( float localTargetX, float localTargetY )
{
  m_LocalTargetX = localTargetX;
  m_LocalTargetY = localTargetY;
}

/** Set local target trajectory before update!
 * @param localTrajecX x-coordinate
 * @param localTrajecY y-coordinate
 */
void
CSelectDriveMode::SetLocalTrajec( float localTrajecX, float localTrajecY )
{
  m_LocalTrajecX = localTrajecX;
  m_LocalTrajecY = localTrajecY;
}

/** Returns the proposed x translation which was previously calculated in Update()
 * @return The proposed translation
 */
float
CSelectDriveMode::GetProposedTranslationX()
{
  return m_ProposedTranslationX;
}

/** Returns the proposed y translation which was previously calculated in Update()
 * @return The proposed translation
 */
float
CSelectDriveMode::GetProposedTranslationY()
{
  return m_ProposedTranslationY;
}

/** Returns the proposed rotation which was previously calculated in Update()
 * @return The proposed rotation
 */
float
CSelectDriveMode::GetProposedRotation()
{
  return m_ProposedRotation;
}

/**
 * Search for the escape drive mode and hands over the given information to the escape drive mode
 * This should just be called if potential-field-escape mode is used!
 * @param occGrid pointer to the occGrid
 * @param roboX   robot position on the grid in x
 * @param roboY   robot position on the grid in y
 */
void
CSelectDriveMode::setGridInformation( CLaserOccupancyGrid* occGrid, int roboX, int roboY )
{
  for ( unsigned int i = 0; i < m_vDriveModeList.size(); i++ ) {
    // drive mode checking
    if ( m_vDriveModeList[i]->GetDriveModeName() == NavigatorInterface::ESCAPE ) {
      ((CEscapePotentialFieldDriveModule*)m_vDriveModeList[i])->setGridInformation( occGrid, roboX, roboY );

      return;
    }
  }
  logger_->log_error("CSelectDriveMode", "Can't find escape drive mode to set grid information");
}

/**
 * Search for the escape drive mode and hands over the given information to the escape drive mode
 * This should just be called if basic-escape mode is used!
 * @param laser_point vector of laser points
 */
void
CSelectDriveMode::setLaserData( std::vector<CEscapeDriveModule::LaserPoint>& laser_point )
{
  for ( unsigned int i = 0; i < m_vDriveModeList.size(); i++ ) {
      // drive mode checking
      if ( m_vDriveModeList[i]->GetDriveModeName() == NavigatorInterface::ESCAPE ) {
        ((CEscapeDriveModule*)m_vDriveModeList[i])->setLaserData( laser_point );

        return;
      }
    }
    logger_->log_error("CSelectDriveMode", "Can't find escape drive mode to set grid information");
}

/* ****************************************************************************** */
/* ****************************************************************************** */
/*                               U P D A T E                                      */
/* ****************************************************************************** */
/* ****************************************************************************** */

/** Pick the drive-mode that should be used and calculate the proposed translation
 * and rotation for the current target (which is set by SetLocalTarget() and
 * SetLocalTrajec(), so make sure to call them beforehand).
 * Update() has to be called before the proposed values are fetched.
 * @param escape Set to true if we want to enter escape-mode
 */
void
CSelectDriveMode::Update( bool escape )
{
  CAbstractDriveMode * m_pDriveMode = 0;
  m_ProposedTranslationX  = 0.;
  m_ProposedTranslationY  = 0.;
  m_ProposedRotation      = 0.;

  // choose the correct drive mode!
  NavigatorInterface::DriveMode desiredMode = NavigatorInterface::MovingNotAllowed;
  if ( escape == true ) {
    if( m_EscapeFlag == 0
     && m_pMotor->GetMotorDesiredTranslationX() != 0
     && m_pMotor->GetMotorDesiredTranslationY() != 0
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
    m_ProposedTranslationX  = 0.;
    m_ProposedTranslationY  = 0.;
    m_ProposedRotation      = 0.;

  } else {
    // valid drive mode!
    // set the values for the drive mode
    m_pDriveMode->SetCurrentRoboPos( m_pMotor->GetCurrentX(),
                                     m_pMotor->GetCurrentY(),
                                     m_pMotor->GetCurrentOri() );

    m_pDriveMode->SetCurrentRoboSpeed( m_pMotor->GetMotorCurrentTranslation(),
                                       m_pMotor->GetMotorCurrentTranslationX(),
                                       m_pMotor->GetMotorCurrentTranslationY(),
                                       m_pMotor->GetMotorCurrentRotation() );

    m_pDriveMode->SetCurrentTarget( m_pColliTarget->dest_x(),
                                    m_pColliTarget->dest_y(),
                                    m_pColliTarget->dest_ori() );

    m_pDriveMode->SetLocalTarget( m_LocalTargetX, m_LocalTargetY );
    m_pDriveMode->SetLocalTrajec( m_LocalTrajecX, m_LocalTrajecY );
    m_pDriveMode->SetCurrentColliMode( m_pColliTarget->orientation_mode(), m_pColliTarget->is_stop_at_target() );

    // update the drive mode
    m_pDriveMode->Update();

    // get the values from the drive mode
    m_ProposedTranslationX  = m_pDriveMode->GetProposedTranslationX();
    m_ProposedTranslationY  = m_pDriveMode->GetProposedTranslationY();
    m_ProposedRotation      = m_pDriveMode->GetProposedRotation();

    // recheck with targetobj maximum settings
    if( (m_pColliTarget->max_velocity() != 0.0)
     && (fabs( m_ProposedTranslationX ) > fabs( m_pColliTarget->max_velocity() )) ) {
      if ( m_ProposedTranslationX > 0.0 )
        m_ProposedTranslationX = m_pColliTarget->max_velocity();
      else
        m_ProposedTranslationX = -m_pColliTarget->max_velocity();
    }

    if( (m_pColliTarget->max_velocity() != 0.0)
     && (fabs( m_ProposedTranslationY ) > fabs( m_pColliTarget->max_velocity() )) ) {
      if ( m_ProposedTranslationY > 0.0 )
        m_ProposedTranslationY = m_pColliTarget->max_velocity();
      else
        m_ProposedTranslationY = -m_pColliTarget->max_velocity();
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
