
/***************************************************************************
 *  forward_omni_drive_mode.cpp - Implementation of drive-mode "forward"
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2014  Tobias Neumann
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

#include "forward_omni_drive_mode.h"
#include <utils/math/common.h>
#include <utils/math/angle.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CForwardOmniDriveModule <plugins/colli/drive_modes/forward_drive_mode.h>
 * This is the SlowForward drive-module, for slow forward only movements.
 */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
CForwardOmniDriveModule::CForwardOmniDriveModule(Logger* logger, Configuration* config)
 : CAbstractDriveMode(logger, config)
{
  logger_->log_debug("CForwardOmniDriveModule", "(Constructor): Entering...");
  m_DriveModeName = NavigatorInterface::Forward;

  m_MaxTranslation = config_->get_float( "/plugins/colli/drive_mode/normal/max_trans" );
  m_MaxRotation    = config_->get_float( "/plugins/colli/drive_mode/normal/max_rot" );

  logger_->log_debug("CForwardOmniDriveModule", "(Constructor): Exiting...");
}


/** Destruct your local values here.
 */
CForwardOmniDriveModule::~CForwardOmniDriveModule()
{
  logger_->log_debug("CForwardOmniDriveModule", "(Destructor): Entering...");
  m_DriveModeName = NavigatorInterface::MovingNotAllowed;
  logger_->log_debug("CForwardOmniDriveModule", "(Destructor): Exiting...");
}

void
CForwardOmniDriveModule::calculateRotation(float ori_alpha_target, float ori_alpha_next_target, float dist_to_target, float angle_next_target)
{
  // first calculate desired angle
  float des_alpha;
  if ( /*dist_to_target >= 0.5 ||*/ isnanf( ori_alpha_next_target ) ) { // at the last 50cm rotate to new angle
    des_alpha = ori_alpha_target;
  } else {
    float angle_min = ori_alpha_target - angle_next_target;
    float angle_max = ori_alpha_target + angle_next_target;
    des_alpha = normalize_mirror_rad( std::max( angle_min, std::min(ori_alpha_next_target, angle_max) ) );
  }

  // then choose rotation speed, depending on desired angle
  const float _TURN_MAX_SPEED_LIMIT_ = M_PI_4;
  if        ( des_alpha > _TURN_MAX_SPEED_LIMIT_ ) {
    m_ProposedRotation = m_MaxRotation;
  } else if ( des_alpha < -_TURN_MAX_SPEED_LIMIT_ ) {
    m_ProposedRotation = -m_MaxRotation;
  } else {
    m_ProposedRotation = des_alpha * ( m_MaxRotation / _TURN_MAX_SPEED_LIMIT_ );
  }
}

void
CForwardOmniDriveModule::calculateTranslation(float dist_to_target, float ori_alpha_target, float dec_factor)
{
  float part_x = 0;
  float part_y = 0;
  if ( ! (m_LocalTargetX == 0 && m_LocalTargetY == 0) ) {
    part_x   = m_LocalTargetX / (fabs(m_LocalTargetX) + fabs(m_LocalTargetY));
    part_y   = m_LocalTargetY / (fabs(m_LocalTargetX) + fabs(m_LocalTargetY));
  }
  m_ProposedTranslationX = part_x * m_MaxTranslation * dec_factor;
  m_ProposedTranslationY = part_y * m_MaxTranslation * dec_factor;

  // Check translation limits
  if ( m_ProposedTranslationX < 0. || fabs(ori_alpha_target) >= M_PI_2 - 0.2 ) {
    m_ProposedTranslationX = 0.;
    m_ProposedTranslationY = 0.;
  }
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
CForwardOmniDriveModule::Update()
{
  m_ProposedTranslationX = 0.0;
  m_ProposedRotation    = 0.0;

  float dist_to_target    = sqrt( sqr(m_LocalTargetX) + sqr(m_LocalTargetY) );
  float alpha_target      = normalize_mirror_rad( atan2( m_LocalTargetY, m_LocalTargetX ) );
  float alpha_next_target = angle_distance(m_RoboOri, m_TargetOri);

  // last time border check............. IMPORTANT!!!
  // because the motorinstructor just tests robots physical borders.
  if ( dist_to_target < 0.04 ) {
    m_ProposedTranslationX = 0.0;
    m_ProposedTranslationY = 0.0;
    m_ProposedRotation     = 0.0;

  } else {

    float angle_tollerance = M_PI_4;
    calculateRotation(alpha_target, alpha_next_target, dist_to_target, angle_tollerance / 2.);

    float dec_factor = 1;
    if ( fabs(alpha_target) >= angle_tollerance ) {                             // if we need to turn a lot => drive slower
      dec_factor = 0.5;
    }

    calculateTranslation(dist_to_target, alpha_target, dec_factor);

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
