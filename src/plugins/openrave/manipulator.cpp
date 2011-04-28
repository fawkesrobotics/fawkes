
/***************************************************************************
 *  manipulator.cpp - Fawkes to OpenRAVE Manipulator Data
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

#include "manipulator.h"

#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenRaveManipulator <plugins/openrave/manipulator.h>
 * Class containing information about all manipulator motors.
 * @author Bahram Maleki-Fard
 */

/** Constructor
 * @param count number of motors of OpenRAVE model
 * @param count_device number of motors of real device
 */
OpenRaveManipulator::OpenRaveManipulator(unsigned int count, unsigned int count_device) :
  __cnt( count ),
  __cnt_device( count_device )
{
}

/** Destructor. */
OpenRaveManipulator::~OpenRaveManipulator()
{
}


/** Adds a motor to the list(vector) of motors
 * @param number motor number in OpenRAVE
 * @param number_device motor number of real device
 */
void
OpenRaveManipulator::add_motor(unsigned int number, unsigned int number_device)
{
  motor_t motor;
  motor.no = number;
  motor.no_device = number_device;
  motor.angle = 0.f;

  __motors.push_back(motor);
}








/* ########## various ########## */
/** Transform single OpenRAVE motor angle to real device angle
 * @param number motor number of real device
 * @param angle motor angle of OpenRAVE model
 * @return transformed angle
 */
float
OpenRaveManipulator::angle_OR_to_device(unsigned int number, float angle) const
{
  // Transformations should be implemented in subclasses, as these depend on
  // the attached manipulator device.
  return angle;
}

/** Transform single device motor angle to OpenRAVE angle
 * @param number motor number of real device
 * @param angle motor angle of real device
 * @return transformed angle
 */
float
OpenRaveManipulator::angle_device_to_OR(unsigned int number, float angle) const
{
  // Transformations should be implemented in subclasses, as these depend on
  // the attached manipulator device.
  return angle;
}

} // end of namespace fawkes