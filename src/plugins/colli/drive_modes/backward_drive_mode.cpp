
/***************************************************************************
 *  backward_drive_mode.cpp - Implementation of drive-mode "backward"
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

#include "backward_drive_mode.h"

#include <utils/math/angle.h>
#include <utils/math/common.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BackwardDriveModule <plugins/colli/drive_modes/backward_drive_mode.h>
 * This is the SlowBackward drive-module, for slow backward only movements.
 */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
BackwardDriveModule::BackwardDriveModule(Logger* logger, Configuration* config)
 : AbstractDriveMode(logger, config)
{
  logger_->log_debug("BackwardDriveModule", "(Constructor): Entering...");
  drive_mode_ = NavigatorInterface::Backward;

  max_trans_ = config_->get_float( "/plugins/colli/drive_mode/normal/max_trans" );
  max_rot_   = config_->get_float( "/plugins/colli/drive_mode/normal/max_rot" );

  logger_->log_debug("BackwardDriveModule", "(Constructor): Exiting");
}


/** Destruct your local values here!
 */
BackwardDriveModule::~BackwardDriveModule()
{
  logger_->log_debug("BackwardDriveModule", "(Destructor): Entering...");
  drive_mode_ = NavigatorInterface::MovingNotAllowed;
  logger_->log_debug("BackwardDriveModule", "(Destructor): Exiting");
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
BackwardDriveModule::backward_curvature( float dist_to_target, float dist_to_trajec, float alpha,
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
BackwardDriveModule::backward_translation( float dist_to_target, float dist_to_front, float alpha,
                                           float cur_trans, float cur_rot, float des_rot)
{
  float des_trans = 0.f;

  if ( fabs( des_rot ) >= 0.f && fabs( des_rot ) <= 1.f )
    des_trans = lin_interpol( fabs( des_rot ), 0.f, 1.f, 0.7f, fabs(max_trans_+0.1f) );

  else if ( fabs( des_rot ) > 1.f )
    des_trans = lin_interpol( fabs( des_rot ), M_PI, 1.f, 0.f, 0.7f );

  // test the borders (no agressive behaviour!)
  if ( des_trans > 0.f ) des_trans = 0.f;
  if ( des_trans < max_trans_ ) des_trans = max_trans_;

  // OLD STUFF
  //   // check stopping on target and compare distances with choosen velocities
  //   if ( fabs( dist_to_target - dist_to_front ) < 0.2 )
  //     {
  //       if (stop_at_target_ == true)
  //  des_trans = min( des_trans, dist_to_target*1.5 );
  //       else
  //  ; // do not stop, so drive behind the target with full power
  //     }
  //   else
  //     {
  //       des_trans = min( des_trans, dist_to_front );
  //     }

  // NEW STUFF
  float trans_target = 10000.f;
  float trans_front  = 10000.f;

  if ( stop_at_target_ == true )
    trans_target = guarantee_trans_stop( dist_to_target, cur_trans, des_trans );

  // And the next collision point
  if ( dist_to_front < dist_to_target )
    trans_front = guarantee_trans_stop( (1.f*dist_to_front)/2.f, cur_trans, des_trans );
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
BackwardDriveModule::update()
{
  proposed_.x = proposed_.y = proposed_.rot = 0.f;

  float dist_to_target = sqrt( sqr(local_target_.x) + sqr(local_target_.y) );
  float alpha          = normalize_mirror_rad(atan2( local_target_.y, local_target_.x ) + M_PI);
  float dist_to_trajec = sqrt( sqr(local_trajec_.x) + sqr(local_trajec_.y) );


  proposed_.rot = backward_curvature( dist_to_target, dist_to_trajec, alpha,
                                      -robot_speed_, -robot_vel_.rot );


  if ( fabs( alpha ) <= M_PI_2+0.1f )
    proposed_.x = backward_translation( dist_to_target, dist_to_trajec, alpha,
                                       -robot_speed_, -robot_vel_.rot, proposed_.rot);

  // last time border check............. IMPORTANT!!!
  // because the motorinstructor just tests robots physical borders.
  if ( dist_to_target >= 0.04f ) {
    proposed_.x  = std::min ( proposed_.x, max_trans_ );
    proposed_.x  = std::max ( proposed_.x, 0.f );
    proposed_.x *= -1.f;

    if (proposed_.rot > max_rot_)
      proposed_.rot = max_rot_;

    if (proposed_.rot < -max_rot_)
      proposed_.rot = -max_rot_;


    if ( !stop_at_target_ && dist_to_target < 1.f ) {
      // reduce rotation velocity to avoid wild rotations
      if ( proposed_.rot > 0.5f )
        proposed_.rot =  0.5f;
      else if ( proposed_.rot < -0.5f )
        proposed_.rot = -0.5f;
    }
  }
}

} // namespace fawkes
