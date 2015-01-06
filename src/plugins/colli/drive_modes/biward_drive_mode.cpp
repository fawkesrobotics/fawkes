
/***************************************************************************
 *  biward_drive_mode.cpp - Implementation of drive-mode "forward + backward"
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

#include "biward_drive_mode.h"
#include "forward_drive_mode.h"
#include "backward_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BiwardDriveModule <plugins/colli/drive_modes/biward_drive_mode.h>
 * This is the SlowBiward drive-module. It is inherited from  the abstract drive mode
 * and uses the other both modes.  If the target is in front, it drives forward
 * to the target, else it drives backward to the target.
 */

/** Constructor.
 * @param forward The Forward drive module
 * @param backward The Backward drive module
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
BiwardDriveModule::BiwardDriveModule( ForwardDriveModule*  forward,
                                      BackwardDriveModule* backward,
                                      Logger* logger,
                                      Configuration* config )
 : AbstractDriveMode(logger, config)
{
  logger_->log_debug("BiwardDriveModule", "(Constructor): Entering...");
  drive_mode_ = NavigatorInterface::AllowBackward;
  mod_forward_  = forward;
  mod_backward_ = backward;

  count_forward_ = 1;

  max_trans_ = config_->get_float( "/plugins/colli/drive_mode/normal/max_trans" );
  max_rot_    = config_->get_float( "/plugins/colli/drive_mode/normal/max_rot" );

  logger_->log_debug("BiwardDriveModule", "(Constructor): Exiting...");
}


/** Destruct your local values here.
 */
BiwardDriveModule::~BiwardDriveModule()
{
  logger_->log_debug("BiwardDriveModule", "(Destructor): Entering...");
  logger_->log_debug("BiwardDriveModule", "(Destructor): Exiting...");
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
BiwardDriveModule::update()
{
  // Just to take care.
  proposed_.x = proposed_.y = proposed_.rot = 0.f;

  // Our drive mode (choose between forward and backward)
  AbstractDriveMode * drive_mode = NULL;

  // search the correct drive mode
  float angle_to_target = atan2( local_target_.y, local_target_.x );

  if ( count_forward_ == 1 && fabs( angle_to_target ) > M_PI_2+0.1 )
    count_forward_ = -1;

  else if ( count_forward_ == 1 )
    count_forward_ = 1;

  else if ( count_forward_ == -1 && fabs( angle_to_target ) < M_PI_2-0.1 )
    count_forward_ = 1;

  else if ( count_forward_ == -1 )
    count_forward_ = -1;

  else {
    logger_->log_debug("BiwardDriveModule", "Undefined state");
    count_forward_ = 0;
  }

  if ( count_forward_ == 1 )
    drive_mode = mod_forward_;
  else
    drive_mode = mod_backward_;

  // set the current info to the drive mode
  drive_mode->set_current_robo_pos( robot_.x, robot_.y, robot_.ori );
  drive_mode->set_current_robo_speed( robot_vel_.x, robot_vel_.y, robot_vel_.rot );
  drive_mode->set_current_target( target_.x, target_.y, target_.ori );
  drive_mode->set_local_target( local_target_.x, local_target_.y );
  drive_mode->set_local_trajec( local_trajec_.x, local_trajec_.y );
  drive_mode->set_current_colli_mode( orient_mode_, stop_at_target_ );

  // update the drive mode
  drive_mode->update();

  // get the values from the drive mode
  proposed_.x   = drive_mode->get_proposed_trans_x();
  proposed_.rot = drive_mode->get_proposed_rot();
}

} // namespace fawkes
