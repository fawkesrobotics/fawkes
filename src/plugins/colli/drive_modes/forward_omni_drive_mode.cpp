
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

#include <cmath>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ForwardOmniDriveModule <plugins/colli/drive_modes/forward_drive_mode.h>
 * This is the SlowForward drive-module, for slow forward only movements.
 */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
ForwardOmniDriveModule::ForwardOmniDriveModule(Logger* logger, Configuration* config)
 : AbstractDriveMode(logger, config)
{
  logger_->log_debug("ForwardOmniDriveModule", "(Constructor): Entering...");
  drive_mode_ = NavigatorInterface::Forward;

  max_trans_ = config_->get_float( "/plugins/colli/drive_mode/normal/max_trans" );
  max_rot_    = config_->get_float( "/plugins/colli/drive_mode/normal/max_rot" );

  logger_->log_debug("ForwardOmniDriveModule", "(Constructor): Exiting...");
}


/** Descturctor. Destruct your local values here. */
ForwardOmniDriveModule::~ForwardOmniDriveModule()
{
  logger_->log_debug("ForwardOmniDriveModule", "(Destructor): Entering...");
  drive_mode_ = NavigatorInterface::MovingNotAllowed;
  logger_->log_debug("ForwardOmniDriveModule", "(Destructor): Exiting...");
}

void
ForwardOmniDriveModule::calculate_rotation(float ori_alpha_target, float ori_alpha_next_target,
                                           float dist_to_target, float angle_allowed_to_next_target)
{
  // first calculate desired angle
  float des_alpha;
  if ( ! std::isfinite( ori_alpha_next_target ) ) {
    des_alpha = ori_alpha_target;
  } else {
    float angle_min = ori_alpha_target - angle_allowed_to_next_target;
    float angle_max = ori_alpha_target + angle_allowed_to_next_target;
    des_alpha = normalize_mirror_rad( std::max( angle_min, std::min(ori_alpha_target, angle_max) ) );
  }

  // then choose rotation speed, depending on desired angle
  const float _TURN_MAX_SPEED_LIMIT_ = M_PI_4;
  if        ( des_alpha > _TURN_MAX_SPEED_LIMIT_ ) {
    proposed_.rot = max_rot_;
  } else if ( des_alpha < -_TURN_MAX_SPEED_LIMIT_ ) {
    proposed_.rot = -max_rot_;
  } else {
    proposed_.rot = des_alpha * ( max_rot_ / _TURN_MAX_SPEED_LIMIT_ );
  }
}

void
ForwardOmniDriveModule::calculate_translation(float dist_to_target, float ori_alpha_target, float dec_factor)
{
  float part_x = 0;
  float part_y = 0;
  if ( ! (local_target_.x == 0 && local_target_.y == 0) ) {
    part_x   = local_target_.x / (fabs(local_target_.x) + fabs(local_target_.y));
    part_y   = local_target_.y / (fabs(local_target_.x) + fabs(local_target_.y));
  }
  proposed_.x = part_x * max_trans_ * dec_factor;
  proposed_.y = part_y * max_trans_ * dec_factor;

  // Check translation limits
  if ( proposed_.x < 0. || fabs(ori_alpha_target) >= M_PI_2 - 0.2 ) {
    proposed_.x = 0.;
    proposed_.y = 0.;
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
ForwardOmniDriveModule::update()
{
  proposed_.x   = 0.f;
  proposed_.rot = 0.f;

  float dist_to_target    = sqrt( sqr(local_target_.x) + sqr(local_target_.y) );
  float alpha_target      = normalize_mirror_rad( atan2( local_target_.y, local_target_.x ) );
  float alpha_next_target = angle_distance_signed(robot_.ori, target_.ori);

  // last time border check............. IMPORTANT!!!
  // because the motorinstructor just tests robots physical borders.
  if ( dist_to_target < 0.04 ) {
    proposed_.x = proposed_.y = proposed_.rot = 0.f;

  } else {
    float angle_tollerance = M_PI_4;
    calculate_rotation(alpha_target, alpha_next_target, dist_to_target, angle_tollerance / 2.);

    float dec_factor = 1;
    if ( fabs(alpha_target) >= angle_tollerance ) {                             // if we need to turn a lot => drive slower
      dec_factor = 0.5;
    }

    calculate_translation(dist_to_target, alpha_target, dec_factor);

    if ( stop_at_target_ ) {
      float target_rel     = std::sqrt( sqr(target_.x - robot_.x) + sqr(target_.y - robot_.y) );
      float robo_trans     = std::sqrt( sqr(robot_vel_.x) + sqr(robot_vel_.y) );
      float proposed_trans = std::sqrt( sqr(proposed_.x) + sqr(proposed_.y) );
      float target_trans   = guarantee_trans_stop(target_rel, robo_trans, proposed_trans);

      float des = fabs(target_trans / proposed_trans);
      if      ( proposed_trans == 0 ) { des = 0; }

      proposed_.x *= des;
      proposed_.y *= des;
    }
  }
}

} // namespace fawkes
