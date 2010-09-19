
/***************************************************************************
 *  motors_katana.cpp - Fawkes to OpenRAVE Katana6M180 Motors Data
 *
 *  Created: Thu Sep 16 14:50:34 2010
 *  Copyright  2010  Bahram Maleki-Fard, AllemaniACs RoboCup Team
 *
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

#include "motors_katana.h"
#include "motors.h"

#include <cmath>
#include <cstdio>

 namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenRAVEMotorsKatana <openrave/motors_katana.h>
 * Class containing information about all katana6M180 motors.
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param count number of motors of OpenRAVE model
 * @param countDevice number of motors of real device
 */
OpenRAVEMotorsKatana::OpenRAVEMotorsKatana(unsigned int count, unsigned int countDevice) :
  OpenRAVEMotors( count, countDevice )
{
}

/** Destructor. */
OpenRAVEMotorsKatana::~OpenRAVEMotorsKatana()
{
}




/* ########## various ######### */
/** Transform single OpenRAVE motor angle to real device angle
 * @param number motor number of real device
 * @param angle motor angle of OpenRAVE model
 * @return transformed angle
 */
float
OpenRAVEMotorsKatana::angleOR2Device(unsigned int number, float angle) const
{
  float _angle;

  switch( number ) {
    case 0:
      _angle = angle + M_PI;
      break;
    case 1:
      _angle = angle;
      break;
    case 2:
      _angle = angle + M_PI;
      break;
    case 3:
      _angle = M_PI/2 - angle;
      break;
    case 4:
      _angle = M_PI - angle; // TODO: use arm-montage angle
      break;
    default:
      _angle = angle;
      break;
  }

  return _angle;
}

/** Transform single device motor angle to OpenRAVE angle
 * @param number motor number of real device
 * @param angle motor angle of real device
 * @return transformed angle
 */
float
OpenRAVEMotorsKatana::angleDevice2OR(unsigned int number, float angle) const
{
  float _angle;

  switch( number ) {
    case 0:
      _angle = angle - M_PI;
      break;
    case 1:
      _angle = angle;
      break;
    case 2:
      _angle = angle - M_PI;
      break;
    case 3:
      _angle = M_PI/2 - angle;
      break;
    case 4:
      _angle = M_PI - angle; // TODO: use arm-montage angle
      break;
    default:
      _angle = angle;
      break;
  }

  return _angle;
}
} // end namespace fawkes