
/***************************************************************************
 *  escape_potential_field_omni_drive_mode.cpp - Implementation of drive-mode "escape"
 *
 *  Created: Tue Mar 25 17:24:18 2014
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

#include "escape_potential_field_omni_drive_mode.h"
#include "../search/og_laser.h"
#include "../common/types.h"

#include <utils/math/angle.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class EscapePotentialFieldOmniDriveModule <plugins/colli/drive_modes/escape_potential_field_omni_drive_mode.h>
 * Class Escape-Drive-Module. This module is called, if an escape is neccessary.
 * It should try to maximize distance to the disturbing obstacle.
 */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
EscapePotentialFieldOmniDriveModule::EscapePotentialFieldOmniDriveModule( Logger* logger, Configuration* config )
 : AbstractDriveMode(logger, config)
{
  logger_->log_debug("EscapePotentialFieldOmniDriveModule", "(Constructor): Entering...");
  drive_mode_ = NavigatorInterface::ESCAPE;
  occ_grid_ = NULL;
  robot_pos_.x = 0.f;
  robot_pos_.y = 0.f;
  turn_ = 0;

  max_trans_ = config_->get_float( "/plugins/colli/drive_mode/escape/max_trans" );
  max_rot_    = config_->get_float( "/plugins/colli/drive_mode/escape/max_rot" );

  cfg_write_spam_debug_ = config_->get_bool( "/plugins/colli/write_spam_debug" );

  logger_->log_debug("EscapePotentialFieldOmniDriveModule", "(Constructor): Exiting...");
}


/** Destruct your local values here.
 */
EscapePotentialFieldOmniDriveModule::~EscapePotentialFieldOmniDriveModule()
{
  logger_->log_debug("EscapePotentialFieldOmniDriveModule", "(Destructor): Entering...");
  logger_->log_debug("EscapePotentialFieldOmniDriveModule", "(Destructor): Exiting...");
}

/**
 * This function sets the Grid information for one escape step
 * @param occ_grid pointer to the occ_grid
 * @param robo_x   robot position on the grid in x
 * @param robo_y   robot position on the grid in y
 */
void
EscapePotentialFieldOmniDriveModule::set_grid_information( LaserOccupancyGrid* occ_grid, int robo_x, int robo_y )
{
  occ_grid_ = occ_grid;
  robot_pos_.x = robo_x;
  robot_pos_.y = robo_y;
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
EscapePotentialFieldOmniDriveModule::update()
{
  static unsigned int cell_cost_occ = occ_grid_->get_cell_costs().occ;

  // This is only called, if we recently stopped...
  if (cfg_write_spam_debug_) {
    logger_->log_debug("EscapePotentialFieldOmniDriveModule", "EscapePotentialFieldOmniDriveModule( update ): Calculating ESCAPING...");
  }

  proposed_.x = proposed_.y = proposed_.rot = 0.f;

  int cell_height = occ_grid_->get_cell_height();
  int cell_width = occ_grid_->get_cell_width();
  int width = occ_grid_->get_width();
  int height = occ_grid_->get_height();

  polar_coord_2d_t target;
  target.r   = 0.1f;
  target.phi = M_PI;
  float target_x = 0.f;
  float target_y = 0.f;

  for (int posX = 0; posX < width; ++posX) {
    for (int posY = 0; posY < height; ++posY) {
      if (occ_grid_->get_prob(posX,posY) >= cell_cost_occ) {
        float dx = float(posX - robot_pos_.x) * cell_height/100;
        float dy = float(posY - robot_pos_.y) * cell_width/100;

        if (dx != 0.f && dy != 0.f) {
          float factor = 1.f / ( (dx*dx + dy*dy) * (dx*dx + dy*dy) );

          target_x -= factor * dx;
          target_y -= factor * dy;
        }
      }
    }
  }

  target.r   = sqrt( target_x*target_x + target_y*target_y );
  target.phi = atan2(target_y, target_x);

  if (cfg_write_spam_debug_) {
    logger_->log_debug("EscapePotentialFieldOmniDriveModule","Target vector: phi: %f\t%f", target.phi, target.r);
  }

  // decide route
  float angle_difference = M_PI_2 - 0.2f;
  float angle     = normalize_mirror_rad(target.phi);
  float angle_abs = fabs( angle );

  bool turn = true;
  float turn_direction  = 0.f;
  float drive_part_x    = 1.f;
  float drive_part_y    = 0.f;

  if ( angle_abs > angle_difference ) {                 // just turn
    turn = true;

    turn_ =  1;
    if (angle < 0.f) {
      turn_direction = -1.f;
    } else {
      turn_direction =  1.f;
    }
  } else {                                              // drive
    turn = false;

    drive_part_x = std::cos( target.phi );
    drive_part_y = std::sin( target.phi );
  }

  if ( turn ) {
    if (cfg_write_spam_debug_) {
      logger_->log_debug("EscapePotentialFieldOmniDriveModule","Turn %f", turn_direction);
    }
    proposed_.rot = turn_direction * max_rot_;
  } else {
    if (cfg_write_spam_debug_) {
      logger_->log_debug("EscapePotentialFieldOmniDriveModule","Drive ( %f , %f )", drive_part_x, drive_part_y);
    }
    proposed_.x = drive_part_x * max_trans_;
    proposed_.y = drive_part_y * max_trans_;
    if ( fabs(turn_direction) > 0.2f ) {
      proposed_.rot = turn_direction * max_rot_;
    }
  }
}


/* ************************************************************************** */
/* ***********************     Private Methods      ************************* */
/* ************************************************************************** */


} // namespace fawkes
