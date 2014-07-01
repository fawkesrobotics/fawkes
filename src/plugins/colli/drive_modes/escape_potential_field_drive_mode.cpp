
/***************************************************************************
 *  escape_potential_field_drive_mode.cpp - Implementation of drive-mode "escape"
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

#include "escape_potential_field_drive_mode.h"
#include "../search/og_laser.h"
#include "../common/types.h"

#include <utils/math/angle.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CEscapePotentialFieldDriveModule <plugins/colli/drive_modes/escape_potential_field_drive_mode.h>
 * Class Escape-Drive-Module. This module is called, if an escape is neccessary.
 * It should try to maximize distance to the disturbing obstacle.
 */

/** Constructor.
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 */
CEscapePotentialFieldDriveModule::CEscapePotentialFieldDriveModule( Logger* logger, Configuration* config )
 : CAbstractDriveMode(logger, config)
{
  logger_->log_debug("CEscapeDriveModule", "(Constructor): Entering...");
  m_DriveModeName = NavigatorInterface::ESCAPE;
  m_pOccGrid = NULL;
  m_robot_pos.x = 0;
  m_robot_pos.y = 0;
  m_turn = 0;

  m_MaxTranslation = config_->get_float( "/plugins/colli/drive_mode/escape/max_trans" );
  m_MaxRotation    = config_->get_float( "/plugins/colli/drive_mode/escape/max_rot" );

  logger_->log_debug("CEscapeDriveModule", "(Constructor): Exiting...");
}


/** Destruct your local values here.
 */
CEscapePotentialFieldDriveModule::~CEscapePotentialFieldDriveModule()
{
  logger_->log_debug("CEscapeDriveModule", "(Destructor): Entering...");
  logger_->log_debug("CEscapeDriveModule", "(Destructor): Exiting...");
}

/**
 * This function sets the Grid information for one escape step
 * @param occGrid pointer to the occGrid
 * @param roboX   robot position on the grid in x
 * @param roboY   robot position on the grid in y
 */
void
CEscapePotentialFieldDriveModule::setGridInformation( CLaserOccupancyGrid* occGrid, int roboX, int roboY )
{
  m_pOccGrid = occGrid;
  m_robot_pos.x = roboX;
  m_robot_pos.y = roboY;
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
 *     m_ProposedTranslation              --> Desired Translation speed
 *     m_ProposedRotation                 --> Desired Rotation speed
 *
 *  Those values are questioned after an Update() was called.
 */
void
CEscapePotentialFieldDriveModule::Update()
{
  static unsigned int cell_cost_occ = m_pOccGrid->get_cell_costs().occ;

  // This is only called, if we recently stopped...
  logger_->log_debug("CEscapeDriveModule", "CEscapeDriveModule( Update ): Calculating ESCAPING...");

  m_ProposedTranslationX  = 0.;
  m_ProposedTranslationY  = 0.;
  m_ProposedRotation      = 0.;

  int cellHeight = m_pOccGrid->getCellHeight();
  int cellWidth = m_pOccGrid->getCellWidth();
  int width = m_pOccGrid->getWidth();
  int height = m_pOccGrid->getHeight();

  polar_coord_2d_t target;
  target.r   = 0.1;
  target.phi = M_PI;
  float target_x = 0;
  float target_y = 0;

  for (int posX = 0; posX < width; ++posX) {
    for (int posY = 0; posY < height; ++posY) {
      if (m_pOccGrid->getProb(posX,posY) >= cell_cost_occ) {
        float dx = float(posX - m_robot_pos.x) * cellHeight/100;
        float dy = float(posY - m_robot_pos.y) * cellWidth/100;

        if (dx != 0 && dy != 0) {
          float factor = 1.0 / ( (dx*dx + dy*dy) * (dx*dx + dy*dy) );

          target_x -= factor * dx;
          target_y -= factor * dy;
        }
      }
    }
  }

  target.r   = sqrt( target_x*target_x + target_y*target_y );
  target.phi = atan2(target_y, target_x);

  logger_->log_warn("CEscapePotentialFieldDriveModule","Target vector: phi: %f\t%f", target.phi, target.r);

  // decide route
  float angle_difference = 0.2;
  float angle     = normalize_mirror_rad(target.phi);
  float angle_abs = fabs( angle );

  bool turn = true;
  float turn_direction  = 0;
  float drive_direction = 0;

  if ( angle_abs > angle_difference/* && angle_abs < M_PI - angle_difference*/ ) {    //just turn
    turn = true;

//    if (angle_abs <= (M_PI_2 + 0.2 * m_turn)) {      //turn to 0
      m_turn =  1.0;
      if (angle < 0) {
        turn_direction = -1.0;
      } else {
        turn_direction =  1.0;
      }
/*    } else {                                        //turn to PI
      m_turn = -1.0;
      if (angle < 0) {
        turn_direction =  1.0;
      } else {
        turn_direction = -1.0;
      }
    }*/
  } else {                                                                        //drive
    turn = false;

//    if (angle_abs <= angle_difference) {                 //forward
      drive_direction =  1.0;
/*    } else if (angle_abs >= M_PI - angle_difference){    //backward
      drive_direction = -1.0;
    } else {
      drive_direction = 0.0;
      logger_->log_error("CEscapePotentialFieldDriveModule","Should drive, but don't know the direction");
    }*/
  }

  if ( turn ) {
    logger_->log_warn("CEscapePotentialFieldDriveModule","Turn %f", turn_direction);
    m_ProposedRotation = turn_direction * m_MaxRotation;
  } else {
    logger_->log_warn("CEscapePotentialFieldDriveModule","Drive %f", drive_direction);
    m_ProposedTranslationX = drive_direction * m_MaxTranslation;
  }

//  if ( angle_abs > angle_difference && angle_abs < M_PI - angle_difference ) {    //just turn
//    if (angle_abs <= M_PI_2) {      //turn to 0
//      logger_->log_warn("CEscapePotentialFieldDriveModule","Turn to 0");
//      if (angle < 0) {
//        logger_->log_warn("CEscapePotentialFieldDriveModule","negative");
//        m_ProposedRotation = -m_MaxRotation;
//      } else {
//        logger_->log_warn("CEscapePotentialFieldDriveModule","positive");
//        m_ProposedRotation =  m_MaxRotation;
//      }
//    } else {                        //turn to PI
//      logger_->log_warn("CEscapePotentialFieldDriveModule","Turn to PI");
//      if (angle < 0) {
//        logger_->log_warn("CEscapePotentialFieldDriveModule","positive");
//        m_ProposedRotation =  m_MaxRotation;
//      } else {
//        logger_->log_warn("CEscapePotentialFieldDriveModule","negative");
//        m_ProposedRotation = -m_MaxRotation;
//      }
//    }
//  } else {                                                                          //drive and turn
//    if (angle_abs <= angle_difference) {                 //forward
//      logger_->log_warn("CEscapePotentialFieldDriveModule","Drive forward");
//      m_ProposedTranslation =  m_MaxTranslation;
//    } else if (angle_abs >= M_PI - angle_difference){    //backward
//      logger_->log_warn("CEscapePotentialFieldDriveModule","Drive backward");
//      m_ProposedTranslation = -m_MaxTranslation;
//    } else {
//      logger_->log_warn("CEscapePotentialFieldDriveModule","Should drive, but don't know the direction");
//    }
//  }

}


/* ************************************************************************** */
/* ***********************     Private Methods      ************************* */
/* ************************************************************************** */


} // namespace fawkes
