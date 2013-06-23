
/***************************************************************************
 *  kinova_jaco.cpp - Fawkes to OpenRAVE Kinova Jaco Manipulator Data
 *
 *  Created: Thu Sep 08 15:34:52 2011
 *  Copyright  2011  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#include "kinova_jaco.h"
#include "../manipulator.h"

#include <utils/math/angle.h>
#include <cmath>
#include <cstdio>

 namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenRaveManipulatorKinovaJaco <plugins/openrave/manipulators/kinova_jaco.h>
 * Class containing information about all Kinova Jaco motors.
 * Basic model is provided by OpenRAVE.
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param count number of motors of OpenRAVE model
 * @param countDevice number of motors of real device
 */
OpenRaveManipulatorKinovaJaco::OpenRaveManipulatorKinovaJaco(unsigned int count, unsigned int countDevice) :
  OpenRaveManipulator( count, countDevice )
{
}

/** Destructor. */
OpenRaveManipulatorKinovaJaco::~OpenRaveManipulatorKinovaJaco()
{
}




/* ########## various ######### */
/** Transform single OpenRAVE motor angle to real device angle
 * @param number motor number of real device
 * @param angle motor angle of OpenRAVE model
 * @return transformed angle
 */
float
OpenRaveManipulatorKinovaJaco::angle_OR_to_device(unsigned int number, float angle) const
{
  float _angle;

  switch( number ) {
    case 0:
      _angle = 1.5*M_PI + rad2deg(angle);
      break;
    case 1:
      _angle = M_PI + rad2deg(angle);
      break;
    case 2:
      _angle = M_PI + rad2deg(angle);
      break;
    case 3:
      _angle = rad2deg(angle);
      break;
    case 4:
      _angle = rad2deg(angle);
      break;
    case 5:
      _angle = rad2deg(angle) + M_PI;
      break;

    default:
      _angle = rad2deg(angle);
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
OpenRaveManipulatorKinovaJaco::angle_device_to_OR(unsigned int number, float angle) const
{
  float _angle;

  switch( number ) {
    case 0:
      _angle = deg2rad(angle) - 1.5*M_PI;
      break;
    case 1:
      _angle = deg2rad(angle) - M_PI;
      break;
    case 2:
      _angle = deg2rad(angle) - M_PI;
      break;
    case 3:
      _angle = deg2rad(angle);
      break;
    case 4:
      _angle = deg2rad(angle);
      break;
    case 5:
      _angle = deg2rad(angle) - M_PI;
      break;
    default:
      _angle = deg2rad(angle);
      break;
  }

  return _angle;
}
} // end namespace fawkes