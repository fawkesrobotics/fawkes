
/***************************************************************************
 *  escape_drive_mode.cpp - Implementation of drive-mode "escape"
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013-2014  Bahram Maleki-Fard
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

#include "escape_drive_mode.h"

#include "../utils/rob/roboshape_colli.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class EscapeDriveModule <plugins/colli/drive_modes/escape_drive_mode.h>
 * Class Escape-Drive-Module. This module is called, if an escape is neccessary.
 * It should try to maximize distance to the disturbing obstacle.
 */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
EscapeDriveModule::EscapeDriveModule( Logger* logger, Configuration* config )
 : AbstractDriveMode(logger, config)
{
  logger_->log_info("EscapeDriveModule", "(Constructor): Entering...");
  drive_mode_ = NavigatorInterface::ESCAPE;

  max_trans_ = config_->get_float( "/plugins/colli/drive_mode/escape/max_trans" );
  max_rot_    = config_->get_float( "/plugins/colli/drive_mode/escape/max_rot" );

  robo_shape_ = new RoboShapeColli( "/plugins/colli/roboshape/", logger, config, 2 );

  logger_->log_info("EscapeDriveModule", "(Constructor): Exiting...");
}


/** Destructor. Destruct your local values here. */
EscapeDriveModule::~EscapeDriveModule()
{
  logger_->log_info("EscapeDriveModule", "(Destructor): Entering...");
  logger_->log_info("EscapeDriveModule", "(Destructor): Exiting...");
}


/* ************************************************************************** */
/* ***********************        U P D A T E       ************************* */
/* ************************************************************************** */

/** Calculate here your desired settings. What you desire is checked afterwards to the current
 *    settings of the physical boundaries, but take care also.
 *
 *  How you do this is up to you, but be careful, our hardware is expensive!!!!
 *
 *  All values of the other drive modes inherited by the abstract-drive-mode are
 *    non-valid, because search did not succeed or should not have been called!
 *    So do not use them. Instead here you use the m_pLaser!
 *
 *  Afterwards filled should be:
 *
*     proposed_          --> Desired translation and rotation speed
 *
 *  Those values are questioned after an update() was called.
 */
void
EscapeDriveModule::update()
{
  // This is only called, if we recently stopped...
  logger_->log_debug("EscapeDriveModule", "EscapeDriveModule( update ): Calculating ESCAPING...");

  proposed_.x = proposed_.y = proposed_.rot = 0.f;

  fill_normalized_readings();
  sort_normalized_readings();

  bool danger_front = check_danger( readings_front_ );
  bool danger_back  = check_danger( readings_back_  );

  bool can_turn_left = turn_left_allowed();
  bool can_turn_right = turn_right_allowed();

  if (danger_front)
    logger_->log_debug("EscapeDriveModule", "DANGER IN FRONT");

  if (danger_back)
    logger_->log_debug("EscapeDriveModule", "DANGER IN BACK");

  if (check_danger(readings_left_front_))
    logger_->log_debug("EscapeDriveModule", "DANGER IN LEFT FRONT");

  if (check_danger(readings_left_back_))
    logger_->log_debug("EscapeDriveModule", "DANGER IN LEFT BACK");

  if (check_danger(readings_right_front_))
    logger_->log_debug("EscapeDriveModule", "DANGER IN RIGHT FRONT");

  if (check_danger(readings_right_back_))
    logger_->log_debug("EscapeDriveModule", "DANGER IN RIGHT BACK");

  if (!can_turn_left)
    logger_->log_debug("EscapeDriveModule", "DANGER IF TURNING LEFT!!!");

  if (!can_turn_right)
    logger_->log_debug("EscapeDriveModule", "DANGER IF TURNING RIGHT!!!");


  if ( danger_front && danger_back && can_turn_right ) {
    proposed_.rot = -max_rot_;

  } else if ( danger_front && danger_back && can_turn_left ) {
    proposed_.rot = max_rot_;

  } else if (!danger_front && danger_back) {
    proposed_.x = max_trans_;

    if ( (can_turn_right) && (local_target_.y <= robot_.y) )
      proposed_.rot =  -max_rot_;
    else if ( (can_turn_left) && (local_target_.y >= robot_.y) )
      proposed_.rot = max_rot_;

  } else if (danger_front && !danger_back) {
    proposed_.x = -max_trans_;

    if ( (can_turn_right) && (local_target_.y <= robot_.y) )
      proposed_.rot =  -max_rot_;
    else if ( (can_turn_left) && (local_target_.y >= robot_.y) )
      proposed_.rot = max_rot_;

  } else if ( !danger_front && !danger_back ) {
    // depending on target coordinates, decide which direction to escape to
    if ( target_.x > robot_.x )
      proposed_.x = max_trans_;
    else
      proposed_.x = -max_trans_;

    if ( (can_turn_right) && (local_target_.y <= robot_.y) )
      proposed_.rot =  -max_rot_;
    else if ( (can_turn_left) && (local_target_.y >= robot_.y) )
      proposed_.rot = max_rot_;
  }
}


/**
 * This function sets the laser points for one escape round
 * @param laser_points vector of laser points
 */
void
EscapeDriveModule::set_laser_data( std::vector<polar_coord_2d_t>& laser_points )
{
  laser_points_ = laser_points;
}


/* ************************************************************************** */
/* ***********************     Private Methods      ************************* */
/* ************************************************************************** */
void
EscapeDriveModule::fill_normalized_readings()
{
  readings_normalized_.clear();

  for ( unsigned int i = 0; i < laser_points_.size(); i++ ) {
    float rad    = normalize_rad( laser_points_.at( i ).phi );
    float sub    = robo_shape_->get_robot_length_for_rad( rad );
    float length = laser_points_.at( i ).r;
    readings_normalized_.push_back( length - sub );
  }
}


void
EscapeDriveModule::sort_normalized_readings()
{
  readings_front_.clear();
  readings_back_.clear();
  readings_left_front_.clear();
  readings_left_back_.clear();
  readings_right_front_.clear();
  readings_right_back_.clear();

  float ang_fl = normalize_rad(robo_shape_->get_angle_front_left());
  float ang_fr = normalize_rad(robo_shape_->get_angle_front_right());
  float ang_bl = normalize_rad(robo_shape_->get_angle_back_left());
  float ang_br = normalize_rad(robo_shape_->get_angle_back_right());
  float ang_ml = normalize_rad(robo_shape_->get_angle_left());
  float ang_mr = normalize_rad(robo_shape_->get_angle_right());

  if(!( (ang_fl < ang_ml) && (ang_ml < ang_bl) && (ang_bl < ang_br)
     &&(ang_br < ang_mr) && (ang_mr < ang_fr) ))
    logger_->log_error("RoboShape", "Angles are bad!!!");

  unsigned int i = 0;
  float rad = 0.f;

  while ( i < laser_points_.size() ) {
    if( laser_points_.at(i).r > 0.01f ) {
      rad = normalize_rad( laser_points_.at(i).phi );

      if( rad < ang_fl || rad >= ang_fr )
        readings_front_.push_back( readings_normalized_[i] );

      else if( rad < ang_ml )
        readings_left_front_.push_back( readings_normalized_[i] );

      else if( rad < ang_bl )
        readings_left_back_.push_back( readings_normalized_[i] );

      else if( rad < ang_br )
        readings_back_.push_back( readings_normalized_[i] );

      else if( rad < ang_mr )
        readings_right_back_.push_back( readings_normalized_[i] );

      else if( rad < ang_fr )
        readings_right_front_.push_back( readings_normalized_[i] );
    }

    ++i;
  }
}


bool
EscapeDriveModule::check_danger( std::vector< float > readings )
{
  // if something is smaller than 5 cm, you have to flee.......
  for ( unsigned int i = 0; i < readings.size(); i++ )
    if ( readings[i] < 0.06f )
      return true;

  return false;
}


bool
EscapeDriveModule::turn_left_allowed()
{
  for ( unsigned int i = 0; i < readings_front_.size(); i++ )
    if ( readings_front_[i] < 0.12f )
      return false;

  for ( unsigned int i = 0; i < readings_right_front_.size(); i++ )
    if ( readings_right_front_[i] < 0.06f )
      return false;

  for ( unsigned int i = 0; i < readings_back_.size(); i++ )
    if ( readings_back_[i] < 0.07f )
      return false;

  for ( unsigned int i = 0; i < readings_left_back_.size(); i++ )
    if ( readings_left_back_[i] < 0.13f )
      return false;

  return true;
}



bool
EscapeDriveModule::turn_right_allowed()
{
  for ( unsigned int i = 0; i < readings_front_.size(); i++ )
    if ( readings_front_[i] < 0.12f )
      return false;

  for ( unsigned int i = 0; i < readings_left_front_.size(); i++ )
    if ( readings_left_front_[i] < 0.06f )
      return false;

  for ( unsigned int i = 0; i < readings_back_.size(); i++ )
    if ( readings_back_[i] < 0.07f )
      return false;

  for ( unsigned int i = 0; i < readings_right_back_.size(); i++ )
    if ( readings_right_back_[i] < 0.13f )
      return false;

  return true;
}

} // namespace fawkes
