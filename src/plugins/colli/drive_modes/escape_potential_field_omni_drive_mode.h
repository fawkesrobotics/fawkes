
/***************************************************************************
 *  escape_potential_field_omni_drive_mode.h - Implementation of drive-mode "escape"
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

#ifndef __PLUGINS_COLLI_ESCAPE_POTENTIAL_FIELD_OMNI_DRIVE_MODE_H_
#define __PLUGINS_COLLI_ESCAPE_POTENTIAL_FIELD_OMNI_DRIVE_MODE_H_

#include "abstract_drive_mode.h"
#include <utils/math/types.h>

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class LaserOccupancyGrid;

class EscapePotentialFieldOmniDriveModule : public AbstractDriveMode
{
 public:
  EscapePotentialFieldOmniDriveModule( Logger* logger, Configuration* config );
  ~EscapePotentialFieldOmniDriveModule();

  void set_grid_information( LaserOccupancyGrid* occ_grid, int robo_x, int robo_y );
  virtual void update();

 private:
  LaserOccupancyGrid*  occ_grid_;
  point_t robot_pos_;

  bool cfg_write_spam_debug_;

  int turn_;
};

} // end namespace fawkes

#endif
