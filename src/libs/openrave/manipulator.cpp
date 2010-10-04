
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

/** @class OpenRAVEManipulator <openrave/motors.h>
 * Class containing information about all manipulator motors.
 * @author Bahram Maleki-Fard
 */

/** Constructor
 * @param count number of motors of OpenRAVE model
 * @param countDevice number of motors of real device
 */
OpenRAVEManipulator::OpenRAVEManipulator(unsigned int count, unsigned int countDevice) :
  __cnt( count ),
  __cntDevice( countDevice )
{
}

/** Destructor. */
OpenRAVEManipulator::~OpenRAVEManipulator()
{
}


/** Adds a motor to the list(vector) of motors
 * @param number motor number in OpenRAVE
 * @param numberDevice motor number of real device
 */
void
OpenRAVEManipulator::addMotor(unsigned int number, unsigned int numberDevice)
{
  motor_t motor;
  motor.no = number;
  motor.noDevice = numberDevice;
  motor.angle = 0.f;

  __motors.push_back(motor);
}




/* ########## getter ########## */
/** Get motor angles of OpenRAVE model
 * @return vector of angles
 */
std::vector<float>
OpenRAVEManipulator::getAngles() const
{
  std::vector<float> _angles(__cnt);
  for (unsigned int i=0; i<__motors.size(); i++) {
    _angles[__motors[i].no] = __motors[i].angle;
  }
  return _angles;
}

/** Get motor angles of real device
 * @return vector of angles
 */
std::vector<float>
OpenRAVEManipulator::getAnglesDevice() const
{
  return anglesOR2Device(getAngles());
}

/** Transform OpenRAVE motor angles to real device angles
 * @param angles motor angles of OpenRAVE model
 * @return vector of angles
 */
std::vector<float>
OpenRAVEManipulator::anglesOR2Device(std::vector<float> angles) const
{
  std::vector<float> _angles(__cntDevice);
  for (unsigned int i=0; i<__motors.size(); i++) {
    _angles[__motors[i].noDevice] = angleOR2Device(__motors[i].noDevice, __motors[i].angle);
  }

  return _angles;
}




/* ########## setter ########## */
/** Set motor angles of OpenRAVE model
 * @param angles motor angles
 */
void
OpenRAVEManipulator::setAngles(std::vector<float> angles)
{
  for (unsigned int i=0; i<__motors.size(); i++) {
    __motors[i].angle = angles[__motors[i].no];
  }
}

/** Set motor angles of real device
 * @param angles motor angles
 */
void
OpenRAVEManipulator::setAnglesDevice(std::vector<float> angles)
{
  for (unsigned int i=0; i<__motors.size(); i++) {
    __motors[i].angle = angleDevice2OR(__motors[i].noDevice, angles[__motors[i].noDevice]);
  }
}



/* ########## various ########## */
/** Transform single OpenRAVE motor angle to real device angle
 * @param number motor number of real device
 * @param angle motor angle of OpenRAVE model
 * @return transformed angle
 */
float
OpenRAVEManipulator::angleOR2Device(unsigned int number, float angle) const
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
OpenRAVEManipulator::angleDevice2OR(unsigned int number, float angle) const
{
  // Transformations should be implemented in subclasses, as these depend on
  // the attached manipulator device.
  return angle;
}

} // end of namespace fawkes