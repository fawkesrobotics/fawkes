
/***************************************************************************
 *  forward_drive_mode.cpp - Implementation of drive-mode "forward"
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

#include "forward_drive_mode.h"
#include <utils/math/common.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ForwardDriveModule <plugins/colli/drive_modes/forward_drive_mode.h>
 * This is the Forward drive-module, for forward only movements.
 */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
ForwardDriveModule::ForwardDriveModule(Logger* logger, Configuration* config)
 : AbstractDriveMode(logger, config)
{
  logger_->log_debug("ForwardDriveModule", "(Constructor): Entering...");
  drive_mode_ = NavigatorInterface::Forward;

  max_trans_ = config_->get_float( "/plugins/colli/drive_mode/normal/max_trans" );
  max_rot_    = config_->get_float( "/plugins/colli/drive_mode/normal/max_rot" );

  logger_->log_debug("ForwardDriveModule", "(Constructor): Exiting...");
}


/** Destructor. Destruct your local values here. */
ForwardDriveModule::~ForwardDriveModule()
{
  logger_->log_debug("ForwardDriveModule", "(Destructor): Entering...");
  drive_mode_ = NavigatorInterface::MovingNotAllowed;
  logger_->log_debug("ForwardDriveModule", "(Destructor): Exiting...");
}


/** Calculate by given variables a new rotation to give for the motor to minimize curvature.
 *
 *  DOC.: This here is the most complicated part in realizing the colli. By the given information
 *        I have to calculate a rotation I want to achieve in an optimal way.
 *        Here this is solved in an interesting way:
 *        First, check how long the curvature is, we want to drive to the target. This is done by
 *        approximating the size of the triangle around this curvature given by collision and
 *        and targetpoint and the angle between those both. Afterwards the time we have to drive
 *        with constant speed is calculated. Now we have the time we want to turn the angle. By
 *        multiplying this with a high constant (here 4), we rotate faster than we want to, but
 *        so the curvatures are not so extraordinary big. Afterwards there is an proportional
 *        part added, so we have a control factor here. (P-Regler ;-) )
 *
 *  @return A desired rotation.
 */
float
ForwardDriveModule::forward_curvature( float dist_to_target, float dist_to_trajec, float alpha,
                                       float cur_trans, float cur_rot )
{
  return 1.2f*alpha;
}


/** Calculate by given variables a new translation to give for the motor to
 *    minimize distance to the target.
 *
 *  DOC.: This here is a fairly easy routine after the previous one ;-). It calculates
 *        to a proposed rotation the translation part. This means, in relation to
 *        distances to target and trajec and the current values we calculate a new one.
 *
 *  @return A desired translation.
 */
float
ForwardDriveModule::forward_translation( float dist_to_target, float dist_to_front, float alpha,
                                         float cur_trans, float cur_rot, float des_rot )
{
  if( fabs(alpha) >= M_PI_2 ) {
    // target is more than +-90° away. Turn without driving first
    return 0.f;
  }

  float des_trans = 0.f;
  /*
  if ( fabs( des_rot ) >= 0.0 && fabs( des_rot ) <= 1.0 )
    des_trans = lin_interpol( fabs( des_rot ), 1.0, 0.0, 0.7, max_trans_+0.1 );
  else if ( fabs( des_rot ) > 1.0 )
    des_trans = lin_interpol( fabs( des_rot ), M_PI, 1.0, 0.0, 0.7 );
  */
  /* We only translate if the target is in angle of +-90° (checked above!)
   * With this interpolation: The higher the rotation, the lower the translation.
   * Why? Because the amount of rotation is related to where the target lies. If it
   * lies ahead, i.e. rotation is low, we can drive faster. If the rotation needs
   * to be high to reach the target, we assume that it is better to drive slower.
   */
  des_trans = lin_interpol( fabs(des_rot), 0.f, M_PI_2, max_trans_, 0.f);

  // OLD STUFF!!!
  //   // check stopping on target and compare distances with choosen velocities
  //   if ( fabs( dist_to_target - dist_to_front ) < 0.2 )
  //     {
  //       if (stop_at_target_ == true)
  //  des_trans = min( des_trans, dist_to_target*1.5 );
  //       else
  //  ;  // do not stop, so drive behind the target with full power
  //     }
  //   else
  //     {
  //       des_trans = min( des_trans, dist_to_front );
  //     }
  // OLD STUFF END HERE


  // NEW STUFF
  float trans_target = 10000.f;
  float trans_front  = 10000.f;

  if ( stop_at_target_ == true )
    trans_target = guarantee_trans_stop( dist_to_target, cur_trans, des_trans );

  // And the next collision point
  if ( dist_to_front > 0.f && dist_to_front < dist_to_target )
    trans_front = guarantee_trans_stop( dist_to_front, cur_trans, des_trans );
  // NEW STUFF END HERE

  des_trans = std::min( des_trans, std::min( trans_target, trans_front ) );

  return des_trans;
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
ForwardDriveModule::update()
{
  proposed_.x = proposed_.y = proposed_.rot = 0.f;

  float dist_to_target = sqrt( sqr(local_target_.x) + sqr(local_target_.y) );
  float alpha          = atan2( local_target_.y, local_target_.x );
  float dist_to_trajec = sqrt( sqr(local_trajec_.x) + sqr(local_trajec_.y) );

  // last time border check............. IMPORTANT!!!
  // because the motorinstructor just tests robots physical borders.
  if ( dist_to_target >= 0.04 ) {
    // Calculate ideal rotation and translation
    proposed_.rot = forward_curvature( dist_to_target, dist_to_trajec, alpha,
                                                robot_speed_, robot_vel_.rot );

    proposed_.x = forward_translation( dist_to_target, dist_to_trajec, alpha,
                                                     robot_speed_, robot_vel_.rot, proposed_.rot );

    // Track relation between proposed-rotation and max-rotation. Use this to adjust the
    // proposed-translation. Only required if the value is smaller than 1, otherwise we are not
    // reducing the proposed-rotation, because it is smaller than max-rotaion
    float trans_correction = fabs( max_rot_ / proposed_.rot );
    if( trans_correction < 1.f ) {
      // for now we simply reduce the translation quadratically to how much the rotation has been reduced
      proposed_.x *= trans_correction * trans_correction;
    }

    // Check rotation limits.
    // Remember, possible reduction of rotation has been considered already (trans_correction)
    if (proposed_.rot >  max_rot_)
      proposed_.rot =  max_rot_;
    else if (proposed_.rot < -max_rot_)
      proposed_.rot = -max_rot_;

    // Check translation limits
    proposed_.x = std::max( 0.f, std::min (proposed_.x, max_trans_) );
    // maybe consider adjusting rotation again, in case we had to reduce the translation
  }
}

} // namespace fawkes
