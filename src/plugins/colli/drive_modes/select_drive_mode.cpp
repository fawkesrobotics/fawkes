
/***************************************************************************
 *  select_drive_mode.cpp - Class that selects drive-mode from a collection
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

#include "../common/types.h"
#include "select_drive_mode.h"

// INCLUDE HERE YOUR DRIVE MODES!!!
#include "stop_drive_mode.h"
#include "escape_drive_mode.h"
#include "escape_potential_field_drive_mode.h"
#include "forward_drive_mode.h"
#include "backward_drive_mode.h"
#include "biward_drive_mode.h"

#include "forward_omni_drive_mode.h"
#include "escape_potential_field_omni_drive_mode.h"
// YOUR CHANGES SHOULD END HERE!!!

#include "../search/og_laser.h"

#include <interfaces/MotorInterface.h>
#include <interfaces/NavigatorInterface.h>
#include <logging/logger.h>
#include <config/config.h>
#include <utils/math/angle.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SelectDriveMode <plugins/colli/drive_modes/select_drive_mode.h>
 * This class selects the correct drive mode and calls the appopriate drive component
 */

/** Constructor.
 * @param motor       The motor interface
 * @param target      The "colli target" NavigatorInterface
 * @param logger      The fawkes logger
 * @param config      The fawkes configuration
 * @param escape_mode The chosen escape mode
 */
SelectDriveMode::SelectDriveMode( MotorInterface* motor,
                                  NavigatorInterface* target,
                                  Logger* logger,
                                  Configuration* config,
                                  colli_escape_mode_t escape_mode )
 : logger_( logger ),
   config_( config ),
   if_colli_target_( target ),
   if_motor_( motor ),
   cfg_escape_mode_( escape_mode ),
   escape_flag_( 0 ) // no escaping at the beginning
{
  logger_->log_debug("SelectDriveMode", "(Constructor): Entering");
  drive_modes_.clear();

  std::string drive_restriction = config->get_string("/plugins/colli/drive_mode/restriction");

  if ( drive_restriction == "omnidirectional" ) {
    drive_restriction_ = colli_drive_restriction_t::omnidirectional;
  } else if ( drive_restriction == "differential" ) {
    drive_restriction_ = colli_drive_restriction_t::differential;
  } else {
    throw fawkes::Exception("Drive restriction '%s' is unknown", drive_restriction.c_str());
  }

  logger_->log_debug("SelectDriveMode", "(Constructor): Creating Drive Mode Objects");

  // Add generic drive modes
  drive_modes_.push_back( (AbstractDriveMode *)new StopDriveModule(logger_, config_) );

  // Add specific drive modes
  if ( drive_restriction_ == colli_drive_restriction_t::omnidirectional ) {
    load_drive_modes_omnidirectional();
  } else {
    load_drive_modes_differential();
  }

  logger_->log_debug("SelectDriveMode", "(Constructor): Exiting");
}

/** Desctructor. */
SelectDriveMode::~SelectDriveMode()
{
  logger_->log_debug("SelectDriveMode", "(Destructor): Entering");
  for ( unsigned int i = 0; i < drive_modes_.size(); i++ )
    delete drive_modes_[i];
  logger_->log_debug("SelectDriveMode", "(Destructor): Exiting");
}


void
SelectDriveMode::load_drive_modes_differential()
{
  // escape drive mode
  if (cfg_escape_mode_ == colli_escape_mode_t::potential_field) {
    drive_modes_.push_back( (AbstractDriveMode *)new EscapePotentialFieldDriveModule( logger_, config_) );
  } else if (cfg_escape_mode_ == colli_escape_mode_t::basic) {
    drive_modes_.push_back( (AbstractDriveMode *)new EscapeDriveModule( logger_, config_) );
  } else {
    logger_->log_error("SelectDriveMode", "Unknown escape drive mode. Using basic as default");
    drive_modes_.push_back( (AbstractDriveMode *)new EscapeDriveModule( logger_, config_) );
  }

  // forward drive mode (have to remember for biward driving!
  ForwardDriveModule* forward = new ForwardDriveModule(logger_, config_);
  drive_modes_.push_back( (AbstractDriveMode *)forward );

  // backward drive mode (have to remember for biward driving!
  BackwardDriveModule* backward = new BackwardDriveModule(logger_, config_);
  drive_modes_.push_back( (AbstractDriveMode *)backward );

  // biward drive mode (takes both forward and backward drive modes as argument!
  drive_modes_.push_back( (AbstractDriveMode *)new BiwardDriveModule(forward, backward, logger_, config_) );
}


void
SelectDriveMode::load_drive_modes_omnidirectional()
{
  // escape drive mode
  if (cfg_escape_mode_ == colli_escape_mode_t::potential_field) {
    drive_modes_.push_back( (AbstractDriveMode *)new EscapePotentialFieldOmniDriveModule( logger_, config_) );
  } else if (cfg_escape_mode_ == colli_escape_mode_t::basic) {
     // CAUTION: This is an differential drive mode!
    drive_modes_.push_back( (AbstractDriveMode *)new EscapeDriveModule( logger_, config_) );
  } else {
    logger_->log_error("SelectDriveMode", "Unknown escape drive mode. Using potential field omni as default");
    drive_modes_.push_back( (AbstractDriveMode *)new EscapePotentialFieldOmniDriveModule( logger_, config_) );
  }

  ForwardOmniDriveModule* forward = new ForwardOmniDriveModule(logger_, config_);
  drive_modes_.push_back( (AbstractDriveMode *)forward );
}


/** Set local target point before update!
 * @param x x-coordinate
 * @param y y-coordinate
 */
void
SelectDriveMode::set_local_target( float x, float y )
{
  local_target_.x = x;
  local_target_.y = y;
}

/** Set local target trajectory before update!
 * @param x x-coordinate
 * @param y y-coordinate
 */
void
SelectDriveMode::set_local_trajec( float x, float y )
{
  local_trajec_.x = x;
  local_trajec_.y = y;
}

/** Returns the proposed x translation which was previously calculated in update()
 * @return The proposed translation
 */
float
SelectDriveMode::get_proposed_trans_x()
{
  return proposed_.x;
}

/** Returns the proposed y translation which was previously calculated in update()
 * @return The proposed translation
 */
float
SelectDriveMode::get_proposed_trans_y()
{
  return proposed_.y;
}

/** Returns the proposed rotation which was previously calculated in update()
 * @return The proposed rotation
 */
float
SelectDriveMode::get_proposed_rot()
{
  return proposed_.rot;
}

/**
 * search for the escape drive mode and hands over the given information to the escape drive mode
 * This should just be called if potential-field-escape mode is used!
 * @param occ_grid pointer to the occ_grid
 * @param robo_x   robot position on the grid in x
 * @param robo_y   robot position on the grid in y
 */
void
SelectDriveMode::set_grid_information( LaserOccupancyGrid* occ_grid, int robo_x, int robo_y )
{
  for ( unsigned int i = 0; i < drive_modes_.size(); i++ ) {
    // drive mode checking
    if ( drive_modes_[i]->get_drive_mode_name() == NavigatorInterface::ESCAPE ) {
      ((EscapePotentialFieldDriveModule*)drive_modes_[i])->set_grid_information( occ_grid, robo_x, robo_y );

      return;
    }
  }
  logger_->log_error("SelectDriveMode", "Can't find escape drive mode to set grid information");
}

/**
 * search for the escape drive mode and hands over the given information to the escape drive mode
 * This should just be called if basic-escape mode is used!
 * @param laser_points vector of laser points
 */
void
SelectDriveMode::set_laser_data( std::vector<polar_coord_2d_t>& laser_points )
{
  for ( unsigned int i = 0; i < drive_modes_.size(); i++ ) {
    // drive mode checking
    if ( drive_modes_[i]->get_drive_mode_name() == NavigatorInterface::ESCAPE ) {
      ((EscapeDriveModule*)drive_modes_[i])->set_laser_data( laser_points );

      return;
    }
  }
  logger_->log_error("SelectDriveMode", "Can't find escape drive mode to set laser information");
}

/* ****************************************************************************** */
/* ****************************************************************************** */
/*                               U P D A T E                                      */
/* ****************************************************************************** */
/* ****************************************************************************** */

/** Pick the drive-mode that should be used and calculate the proposed translation
 * and rotation for the current target (which is set by set_local_target() and
 * set_local_trajec(), so make sure to call them beforehand).
 * update() has to be called before the proposed values are fetched.
 * @param escape Set to true if we want to enter escape-mode
 */
void
SelectDriveMode::update( bool escape )
{
  AbstractDriveMode * drive_mode = NULL;
  proposed_.x = proposed_.y = proposed_.rot = 0.f;

  // choose the correct drive mode!
  NavigatorInterface::DriveMode desired_mode = NavigatorInterface::MovingNotAllowed;
  if ( escape == true ) {
    if( escape_flag_ == 0
     && if_motor_->des_vx() != 0.f
     && if_motor_->des_vx() != 0.f
     && if_motor_->des_omega() != 0.f ) {
      desired_mode = NavigatorInterface::MovingNotAllowed;
      // we have not yet stopped!

    } else {
      // we have stopped recently, so do escape!
      escape_flag_ = 1;
      desired_mode = NavigatorInterface::ESCAPE;
    }

  } else {
    escape_flag_ = 0;
    desired_mode  = if_colli_target_->drive_mode();
  }

  // now search this specific drive mode in the list
  for ( unsigned int i = 0; i < drive_modes_.size(); i++ ) {
    // error checking
    if ( drive_modes_[i]->get_drive_mode_name() == desired_mode
     &&  drive_mode != 0 ) {
      logger_->log_error("SelectDriveMode", "Error while selecting drive mode. There is more than one mode with the same name!!! Stopping!");
      drive_mode = 0;
      break;
    }

    // drive mode checking
    if ( drive_modes_[i]->get_drive_mode_name() == desired_mode
     &&  drive_mode == 0 ) {
      drive_mode = drive_modes_[i];
    }
  }


  if ( drive_mode == 0 ) {
    // invalid pointer
    logger_->log_error("SelectDriveMode", "INVALID DRIVE MODE POINTER, stopping!");
    proposed_.x = proposed_.y = proposed_.rot = 0.f;

  } else {
    // valid drive mode!
    // set the values for the drive mode
    drive_mode->set_current_robo_pos( if_motor_->odometry_position_x(),
                                      if_motor_->odometry_position_y(),
                                      normalize_mirror_rad(if_motor_->odometry_orientation()) );

    drive_mode->set_current_robo_speed( if_motor_->vx(),
                                        if_motor_->vy(),
                                        if_motor_->omega() );

    drive_mode->set_current_target( if_colli_target_->dest_x(),
                                    if_colli_target_->dest_y(),
                                    if_colli_target_->dest_ori() );

    drive_mode->set_local_target( local_target_.x, local_target_.y );
    drive_mode->set_local_trajec( local_trajec_.x, local_trajec_.y );
    drive_mode->set_current_colli_mode( if_colli_target_->orientation_mode(), if_colli_target_->is_stop_at_target() );

    // update the drive mode
    drive_mode->update();

    // get the values from the drive mode
    proposed_.x   = drive_mode->get_proposed_trans_x();
    proposed_.y   = drive_mode->get_proposed_trans_y();
    proposed_.rot = drive_mode->get_proposed_rot();

    // recheck with targetobj maximum settings
    if( (if_colli_target_->max_velocity() != 0.0)
     && (fabs( proposed_.x ) > fabs( if_colli_target_->max_velocity() )) ) {
      if ( proposed_.x > 0.0 )
        proposed_.x = if_colli_target_->max_velocity();
      else
        proposed_.x = -if_colli_target_->max_velocity();
    }

    if( (if_colli_target_->max_velocity() != 0.0)
     && (fabs( proposed_.y ) > fabs( if_colli_target_->max_velocity() )) ) {
      if ( proposed_.y > 0.0 )
        proposed_.y = if_colli_target_->max_velocity();
      else
        proposed_.y = -if_colli_target_->max_velocity();
    }

    if( ( if_colli_target_->max_rotation() != 0.0 )
     && (fabs( proposed_.rot ) > fabs( if_colli_target_->max_rotation() )) ) {
      if ( proposed_.rot > 0.0 )
        proposed_.rot = if_colli_target_->max_rotation();
      else
        proposed_.rot = -if_colli_target_->max_rotation();
    }

  }
}

} // namespace fawkes
