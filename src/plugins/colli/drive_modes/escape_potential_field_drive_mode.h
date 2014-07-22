
/***************************************************************************
 *  escape_potential_drive_mode.h - Escaping mode using potential field
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

#ifndef __PLUGINS_COLLI_ESCAPE_POTENTIAL_FIELD_DRIVE_MODE_H_
#define __PLUGINS_COLLI_ESCAPE_POTENTIAL_FIELD_DRIVE_MODE_H_

#include "abstract_drive_mode.h"
#include <utils/math/types.h>

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CLaserOccupancyGrid;

class CEscapePotentialFieldDriveModule : public CAbstractDriveMode
{
 public:

  CEscapePotentialFieldDriveModule( Logger* logger, Configuration* config );
  ~CEscapePotentialFieldDriveModule();

  void setGridInformation( CLaserOccupancyGrid* occGrid, int roboX, int roboY );
  virtual void Update();

 private:

  /// our pointer to the laserinterface.... lets escape ;-)
  CLaserOccupancyGrid*  m_pOccGrid;
  point_t m_robot_pos;

  /// absolute values are the maximum values. do not act faster!
  float m_MaxTranslation;
  float m_MaxRotation;
  bool cfg_write_spam_debug;

  int   m_turn;
};

} // end namespace fawkes

#endif
