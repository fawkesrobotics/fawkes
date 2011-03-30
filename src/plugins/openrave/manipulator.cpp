
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

/** @class OpenRAVEManipulator <plugins/openrave/manipulator.h>
 * Class containing information about all manipulator motors.
 * @author Bahram Maleki-Fard
 */

/** Constructor
 * @param count number of motors of OpenRAVE model
 * @param count_device number of motors of real device
 */
OpenRAVEManipulator::OpenRAVEManipulator(unsigned int count, unsigned int count_device) :
  __cnt( count ),
  __cnt_device( count_device )
{
}

/** Destructor. */
OpenRAVEManipulator::~OpenRAVEManipulator()
{
}


/** Adds a motor to the list(vector) of motors
 * @param number motor number in OpenRAVE
 * @param number_device motor number of real device
 */
void
OpenRAVEManipulator::add_motor(unsigned int number, unsigned int number_device)
{
  motor_t motor;
  motor.no = number;
  motor.no_device = number_device;
  motor.angle = 0.f;

  __motors.push_back(motor);
}




/* ########## getter ########## */
/** Get motor angles of OpenRAVE model
 * @param to target tvector of angles
 */
void
OpenRAVEManipulator::get_angles(std::vector<float>& to) const
{
  to.resize(__cnt);
  for (unsigned int i=0; i<__motors.size(); i++) {
    to[__motors[i].no] = __motors[i].angle;
  }
}

/** Get motor angles of real device
 * @param to target vector of angles
 */
void
OpenRAVEManipulator::get_angles_device(std::vector<float>& to) const
{
  std::vector<float> tmp;
  get_angles(tmp);
  to = angles_or_to_device(tmp);
}

/** Transform OpenRAVE motor angles to real device angles
 * @param from motor angles of OpenRAVE model
 * @return vector of angles
 */
std::vector<float>
OpenRAVEManipulator::angles_or_to_device(std::vector<float>& from) const
{
  std::vector<float> _to(__cnt_device);
  for (unsigned int i=0; i<__motors.size(); i++) {
    _to[__motors[i].no_device] = angle_OR_to_device(__motors[i].no_device, from[__motors[i].no]);
  }

  return _to;
}




/* ########## setter ########## */
/** Set motor angles of OpenRAVE model
 * @param angles motor angles
 */
void
OpenRAVEManipulator::set_angles(std::vector<float>& angles)
{
  for (unsigned int i=0; i<__motors.size(); i++) {
    __motors[i].angle = angles[__motors[i].no];
  }
}

/** Set motor angles of real device
 * @param angles motor angles
 */
void
OpenRAVEManipulator::set_angles_device(std::vector<float>& angles)
{
  for (unsigned int i=0; i<__motors.size(); i++) {
    __motors[i].angle = angle_device_to_OR(__motors[i].no_device, angles[__motors[i].no_device]);
  }
}



/* ########## various ########## */
/** Transform single OpenRAVE motor angle to real device angle
 * @param number motor number of real device
 * @param angle motor angle of OpenRAVE model
 * @return transformed angle
 */
float
OpenRAVEManipulator::angle_OR_to_device(unsigned int number, float angle) const
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
OpenRAVEManipulator::angle_device_to_OR(unsigned int number, float angle) const
{
  // Transformations should be implemented in subclasses, as these depend on
  // the attached manipulator device.
  return angle;
}

} // end of namespace fawkes