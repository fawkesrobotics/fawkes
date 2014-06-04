
/***************************************************************************
 *  slow_forward_drive_mode.cpp - Implementation of drive-mode "slow forward"
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

#include "slow_forward_drive_mode_omni.h"
#include <utils/math/common.h>
#include <utils/math/angle.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CSlowForwardDriveModule <plugins/colli/drive_modes/slow_forward_drive_mode.h>
 * This is the SlowForward drive-module, for slow forward only movements.
 */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
CSlowForwardOmniDriveModule::CSlowForwardOmniDriveModule(Logger* logger, Configuration* config)
 : CAbstractDriveMode(logger, config)
{
  logger_->log_debug("CSlowForwardDriveModule", "(Constructor): Entering...");
  m_DriveModeName = NavigatorInterface::SlowForward;

  m_MaxTranslation = config_->get_float( "/plugins/colli/drive_mode/slow/max_trans" );
  m_MaxRotation    = config_->get_float( "/plugins/colli/drive_mode/slow/max_rot" );

  logger_->log_debug("CSlowForwardDriveModule", "(Constructor): Exiting...");
}


/** Destruct your local values here.
 */
CSlowForwardOmniDriveModule::~CSlowForwardOmniDriveModule()
{
  logger_->log_debug("CSlowForwardDriveModule", "(Destructor): Entering...");
  m_DriveModeName = NavigatorInterface::MovingNotAllowed;
  logger_->log_debug("CSlowForwardDriveModule", "(Destructor): Exiting...");
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
CSlowForwardOmniDriveModule::Update()
{
  m_ProposedTranslationX = 0.0;
  m_ProposedRotation    = 0.0;

  float dist_to_target = sqrt( sqr(m_LocalTargetX) + sqr(m_LocalTargetY) );
  float alpha_target   = normalize_mirror_rad( atan2( m_LocalTargetY, m_LocalTargetX ) );
//  float alpha_next     = normalize_mirror_rad( atan2( m_TargetY, m_TargetX ) );
//  float dist_to_trajec = sqrt( sqr(m_LocalTrajecX) + sqr(m_LocalTrajecY) );

  // last time border check............. IMPORTANT!!!
  // because the motorinstructor just tests robots physical borders.
  if ( dist_to_target < 0.04 ) {
    m_ProposedTranslationX = 0.0;
    m_ProposedTranslationY = 0.0;
    m_ProposedRotation     = 0.0;

  } else {
    float part_x   = m_LocalTargetX / (fabs(m_LocalTargetX) + fabs(m_LocalTargetY));
    float part_y   = m_LocalTargetY / (fabs(m_LocalTargetX) + fabs(m_LocalTargetY));
    // Calculate ideal rotation and translation
    m_ProposedTranslationX = part_x * m_MaxTranslation;
    m_ProposedTranslationY = part_y * m_MaxTranslation;

    m_ProposedRotation = alpha_target;
//    if ( dist_to_target >= 0.5 ) {         // at the last 50cm rotate to new angle
//      m_ProposedRotation = alpha_target;
//    } else {
//      float angle_min = normalize_mirror_rad( alpha_target - M_PI_4 );
//      float angle_max = normalize_mirror_rad( alpha_target + M_PI_4 );
//      m_ProposedRotation = std::max( angle_min, std::min(alpha_next, angle_max) );
//    }

    if        ( m_ProposedRotation > 0.1 ) {
      m_ProposedRotation = m_MaxRotation;
    } else if ( m_ProposedRotation < -0.1 ) {
      m_ProposedRotation = -m_MaxRotation;
    } else {
      m_ProposedRotation *= ( m_MaxRotation / 0.1 );
    }

//    // Check translation limits
    if ( m_ProposedTranslationX < 0. || fabs(alpha_target) >= M_PI_2 - 0.2 ) {
      m_ProposedTranslationX = 0.;
      m_ProposedTranslationY = 0.;
    }

    if ( m_StopAtTarget ) {
      float target_rel    = std::sqrt( sqr(m_TargetX - m_RoboX) + sqr(m_TargetY - m_RoboY) );
      float roboTrans     = std::sqrt( sqr(m_RoboTransX) + sqr(m_RoboTransY) );
      float proposedTrans = std::sqrt( sqr(m_ProposedTranslationX) + sqr(m_ProposedTranslationY) );
      float targetTrans   = GuaranteeTransStop(target_rel, roboTrans, proposedTrans);

      float des = fabs(targetTrans / proposedTrans);
      if      ( proposedTrans == 0 ) { des = 0; }

      m_ProposedTranslationX *= des;
      m_ProposedTranslationY *= des;
    }
  }
}

} // namespace fawkes
