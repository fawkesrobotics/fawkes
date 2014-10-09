
/***************************************************************************
 *  select_drive_mode.h - Class that selects the drive-mode from a collection
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

#ifndef __PLUGINS_COLLI_SELECT_DRIVE_MODE_H_
#define __PLUGINS_COLLI_SELECT_DRIVE_MODE_H_

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class AbstractDriveMode;
class MotorInterface;
class NavigatorInterface;
class Logger;
class Configuration;
class LaserOccupancyGrid;

class SelectDriveMode
{
public:
  SelectDriveMode( MotorInterface* motor,
                   NavigatorInterface* colli_target,
                   Logger* logger,
                   Configuration* config,
                   colli_escape_mode_t escape_mode = colli_escape_mode_t::basic);
  ~SelectDriveMode( );

  ///\brief Set local target point before update!
  void set_local_target( float x, float y );

  ///\brief Set local trajectory point before update!
  void set_local_trajec( float x, float y );

  ///\brief Has to be called before the proposed values are called.
  void update( bool escape = false );

  ///\brief Returns the proposed translation. After an update.
  float get_proposed_trans_x();

  ///\brief Returns the proposed translation. After an update.
  float get_proposed_trans_y();

  ///\brief Returns the proposed rotation. After an update.
  float get_proposed_rot();

  void set_grid_information( LaserOccupancyGrid* occ_grid, int robo_x, int robo_y );

  void set_laser_data( std::vector<fawkes::polar_coord_2d_t>& laser_points );

private:
  Logger*        logger_;
  Configuration* config_;

  // local pointers to interfaces
  NavigatorInterface*  if_colli_target_;
  MotorInterface*      if_motor_;

  colli_escape_mode_t cfg_escape_mode_;

  // Vector of drive modes
  std::vector< AbstractDriveMode * > drive_modes_;

  // local copies of current local target values
  cart_coord_2d_t local_target_;
  cart_coord_2d_t local_trajec_;

  // local copies of the proposed values
  colli_trans_rot_t proposed_;

  // an escape flag
  int escape_flag_;

  colli_drive_restriction_t drive_restriction_;

  /* ************************************************************************ */
  /* PRIVATE METHODS                                                          */
  /* ************************************************************************ */

  void load_drive_modes_differential();
  void load_drive_modes_omnidirectional();
};

} // namespace fawkes

#endif
