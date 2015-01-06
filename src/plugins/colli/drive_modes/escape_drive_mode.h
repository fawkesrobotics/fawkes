
/***************************************************************************
 *  escape_drive_mode.h - Implementation of drive-mode "escape"
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

#ifndef __PLUGINS_COLLI_ESCAPE_DRIVE_MODE_H_
#define __PLUGINS_COLLI_ESCAPE_DRIVE_MODE_H_

#include "abstract_drive_mode.h"

#include <utils/math/types.h>

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RoboShapeColli;

class EscapeDriveModule : public AbstractDriveMode
{
 public:
  EscapeDriveModule( Logger* logger, Configuration* config );
  ~EscapeDriveModule();

  virtual void update();

  void set_laser_data( std::vector<polar_coord_2d_t>& laser_points );

 private:
  std::vector<polar_coord_2d_t> laser_points_;

  RoboShapeColli*  robo_shape_;

  /// Readings without robolength in it
  std::vector< float > readings_normalized_;
  std::vector< float > readings_front_, readings_back_;
  std::vector< float > readings_left_front_,  readings_left_back_;
  std::vector< float > readings_right_front_, readings_right_back_;


  void fill_normalized_readings();
  void sort_normalized_readings();

  bool check_danger( std::vector< float > readings );
  bool turn_left_allowed();
  bool turn_right_allowed();
};

} // end namespace fawkes

#endif
