
/***************************************************************************
 *  abstract_drive_mode.h - Abstract base class for a drive-mode
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

#ifndef __PLUGINS_COLLI_ABSTRACT_DRIVE_MODE_H_
#define __PLUGINS_COLLI_ABSTRACT_DRIVE_MODE_H_

#include "../common/types.h"

#include <interfaces/NavigatorInterface.h>
#include <logging/logger.h>
#include <config/config.h>

#include <cmath>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class AbstractDriveMode <plugins/colli/drive_modes/abstract_drive_mode.h>
 * This is the base class which calculates drive modes. Drive modes are the
 * proposed settings for the drive-realization out of the found search things.
 */
class AbstractDriveMode
{
 public:
  AbstractDriveMode(Logger* logger, Configuration* config);
  virtual ~AbstractDriveMode();

  ///\brief Sets the current target.
  void set_current_target( float x, float y, float ori );

  ///\brief Sets the current robo position.
  void set_current_robo_pos( float x, float y, float ori );

  ///\brief Sets the current robo speed.
  void set_current_robo_speed( float x, float y, float rot );

  ///\brief Set the colli mode values for each drive mode.
  void set_current_colli_mode( NavigatorInterface::OrientationMode orient, bool stop );

  ///\brief Set the local targetpoint found by the search.
  void set_local_target( float x, float y );

  ///\brief  Set the local trajectory point found by the search.
  void set_local_trajec( float x, float y );

  ///\brief Returns the drive modes name.
  NavigatorInterface::DriveMode get_drive_mode_name();


  ///\brief Calculate the proposed settings which are asked for afterwards.
  virtual void update() = 0;

  ///\brief Returns the proposed x translation
  float get_proposed_trans_x();

  ///\brief Returns the proposed y translation
  float get_proposed_trans_y();

  ///\brief Returns the proposed rotatio
  float get_proposed_rot();


 protected:

  ///\brief Perform linear interpolation
  float lin_interpol( float x, float left, float right, float bot, float top );

  ///\brief Get velocity that guarantees a stop for a given distance
  float guarantee_trans_stop( float distance, float current_trans, float desired_trans );

  field_pos_t target_; /**< current target */
  field_pos_t robot_;  /**< current robot pos */

  colli_trans_rot_t robot_vel_; /**< current robot velocity */
  float robot_speed_;  /**< current robo translation velocity */

  cart_coord_2d_t local_target_; /**< local target */
  cart_coord_2d_t local_trajec_; /**< local trajectory */

  NavigatorInterface::OrientationMode orient_mode_; /**< orient mode of nav if */
  bool stop_at_target_;   /**< flag if stopping on or after target */

  colli_trans_rot_t proposed_;  /**< proposed translation and rotation for next timestep */

  NavigatorInterface::DriveMode drive_mode_;  /**< the drive mode name */

  Logger* logger_;          /**< The fawkes logger */
  Configuration* config_;   /**< The fawkes configuration */

  float max_trans_; /**< The maximum translation speed */
  float max_rot_;   /**< The maximum rotation speed */

private:

  float max_trans_acc_;
  float max_trans_dec_;
  float max_rot_acc_;
  float max_rot_dec_;
  int   frequency_;

  float stopping_distance_;
  float stopping_factor_;
};



/* ************************************************************************** */
/* ***********************  IMPLEMENTATION DETAILS  ************************* */
/* ************************************************************************** */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
inline
AbstractDriveMode::AbstractDriveMode(Logger* logger, Configuration* config)
 : logger_( logger ),
   config_( config )
{
  logger_->log_debug("AbstractDriveMode", "(Constructor): Entering...");
  proposed_.x = proposed_.y = proposed_.rot = 0.f;
  drive_mode_ = NavigatorInterface::MovingNotAllowed;

  // read max_trans_dec_ and max_rot_dec_
  max_trans_acc_ = /*0.75* */config_->get_float("/plugins/colli/motor_instruct/trans_acc");
  max_trans_dec_ = /*0.75* */config_->get_float("/plugins/colli/motor_instruct/trans_dec");
  max_rot_acc_   = /*0.75* */config_->get_float("/plugins/colli/motor_instruct/rot_acc");
  max_rot_dec_   = /*0.75* */config_->get_float("/plugins/colli/motor_instruct/rot_dec");

  stopping_distance_ = config_->get_float("/plugins/colli/drive_mode/stopping_adjustment/distance_addition");
  stopping_factor_   = config_->get_float("/plugins/colli/drive_mode/stopping_adjustment/deceleration_factor");
  stopping_factor_   = std::min(1.f, std::max(0.f, stopping_factor_));

  frequency_ = config_->get_int("/plugins/colli/frequency");

  logger_->log_debug("AbstractDriveMode", "(Constructor): Exiting...");
}


/** Desctructor. */
inline
AbstractDriveMode::~AbstractDriveMode()
{
  logger_->log_debug("AbstractDriveMode", "(Destructor): Entering...");
  logger_->log_debug("AbstractDriveMode", "(Destructor): Exiting...");
}


/** Returns the proposed x translation which was calculated previously in
 *  'update()' which has to be implemented!
 * @return The proposed translation
 */
inline float
AbstractDriveMode::get_proposed_trans_x()
{
  return proposed_.x;
}

/** Returns the proposed y translation which was calculated previously in
 *  'update()' which has to be implemented!
 * @return The proposed translation
 */
inline float
AbstractDriveMode::get_proposed_trans_y()
{
  return proposed_.y;
}

/** Returns the proposed rotation which was calculated previously in
 *  'update()' which has to be implemented!
 * @return The proposed rotation
 */
inline float
AbstractDriveMode::get_proposed_rot()
{
  return proposed_.rot;
}


/** Sets the current target.
 *  Has to be set before update!
 * @param x The target x position
 * @param y The target y position
 * @param ori The target orientation
 */
inline void
AbstractDriveMode::set_current_target( float x, float y, float ori )
{
  target_.x   = x;
  target_.y   = y;
  target_.ori = ori;
}

/** Sets the current robo position.
 *  Has to be set before update!
 * @param x The robot x position
 * @param y The robot y position
 * @param ori The robot orientation
 */
inline void
AbstractDriveMode::set_current_robo_pos( float x, float y, float ori )
{
  robot_.x   = x;
  robot_.y   = y;
  robot_.ori = ori;
}

/** Sets the current robo speed.
 *  Has to be set before update!
 * @param x The robot translation velocity in x-direction only
 * @param y The robot translation velocity in y-direction only
 * @param rot The robot rotation velocity
 */
inline void
AbstractDriveMode::set_current_robo_speed( float x, float y, float rot )
{
  robot_vel_.x   = x;
  robot_vel_.y   = y;
  robot_vel_.rot = rot;
  robot_speed_  = sqrt(x*x + y*y);
  if( x < 0 )
    robot_speed_ = -robot_speed_;
}

/** Set the colli mode values for each drive mode.
 *  Has to be set before update!
 * @param orient Orient at target after target position reached?
 * @param stop Stop at target position?
 */
inline void
AbstractDriveMode::set_current_colli_mode( NavigatorInterface::OrientationMode orient, bool stop )
{
  orient_mode_    = orient;
  stop_at_target_  = stop;
}


/** Set the local targetpoint found by the search.
 *  Has to be set before update!
 * @param x The local target x position
 * @param y The local target y position
 */
inline void
AbstractDriveMode::set_local_target( float x, float y )
{
  local_target_.x = x;
  local_target_.y = y;
}

/** Set the local trajectory point found by the search.
 *  Has to be set before update!
 * @param x The local target x trajectory
 * @param y The local target y trajectory
 */
inline void
AbstractDriveMode::set_local_trajec( float x, float y )
{
  local_trajec_.x = x;
  local_trajec_.y = y;
}

/** Get the drive mode.
 *  Has to be set in the constructor of your drive mode!
 * @return The drive mode
 */
inline NavigatorInterface::DriveMode
AbstractDriveMode::get_drive_mode_name()
{
  return drive_mode_;
}

/** Performs linear interpolation
 * @param x  x
 * @param x1 x1
 * @param x2 x2
 * @param y1 y1
 * @param y2 y2
 * @return interpolated value
 */
inline float
AbstractDriveMode::lin_interpol( float x, float x1, float x2,
                                 float y1, float y2 )
{
  return ( ((x-x1)*(y2-y1))/(x2-x1) + y1 );
}

/** Get velocity that guarantees a stop for a given distance
 * @param distance The distance to stop at
 * @param current_trans Robot's current translation velocity
 * @param desired_trans Robot's currently desired translation velocity
 * @return Proposed translation velocity to stop at given distance
 */
inline float
AbstractDriveMode::guarantee_trans_stop( float distance,
                                         float current_trans,
                                         float desired_trans )
{
  distance = fabs( distance );
  current_trans = fabs( current_trans );

  if ( distance < 0.05f )
    return 0.f;

  if ( current_trans < 0.05f )
    return desired_trans;

  // calculate riemann integral to get distance until robot is stoped
  float trans_tmp         = current_trans;
  float distance_to_stop  = stopping_distance_;
  for (int loops_to_stop = 0; trans_tmp > 0; loops_to_stop++) {
    distance_to_stop += trans_tmp / frequency_;           //First calculate sum (Untersumme)
    trans_tmp        -= max_trans_dec_ * stopping_factor_;  //Then decrease tmp speed
  }

//  logger_->log_debug("AbstractDriveMode","guarantee_trans_stop: distance needed to stop - distance to goal: %f - %f = %f", distance_to_stop, distance, distance_to_stop - distance);
  if (distance_to_stop >= distance) {
    return 0.f;
  } else {
    return desired_trans;
  }
//
//  // dividing by 10 because we're called at 10Hz (TODO: use config value!!)
//  int time_needed_to_distance = (int)( distance / (current_trans/10.0) );
//
//  /* (changes made during AdoT)
//   * 0.1 is an empirical value causing the expected deceleration while
//   * calculating with a slower one. This is likely the difference between what
//   * the colli wants and the motor actually does.
//   * We also add 1, to start deceleration 1 step earlier than the calculation
//   * suggests. Tests showed better results. We could probably skip this, if we
//   * had a proper calculation of velocity adjustment...
//   */
//  int time_needed_to_stop = (int)( current_trans / (0.1*max_trans_dec_) ) +1;
//
//  if( time_needed_to_stop >= time_needed_to_distance ) {
//    float value = std::max( 0.f, current_trans - max_trans_dec_ );
//    return value;
//  } else {
//    float value = std::min( current_trans + max_trans_acc_, desired_trans );
//    // Use this if you are very cautions:
//    //float value = std::min( current_trans + std::min(max_trans_dec_, max_trans_acc_), desired_trans );
//    return value;
//  }
}

} // end namespace fawkes

#endif
