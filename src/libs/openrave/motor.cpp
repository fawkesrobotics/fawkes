
/***************************************************************************
 *  motor.cpp - Fawkes to OpenRAVE Motor Data
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

 #include "motor.h"

 namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenRAVEMotor <openrave/motor.h>
 * Data container of a robot arm motor.
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param number motor number
 */
OpenRAVEMotor::OpenRAVEMotor(unsigned int number) :
  __no( number ),
  __angle( 0.f )
{
}

/** Destructor. */
OpenRAVEMotor::~OpenRAVEMotor()
{
}




/* ########## getter ########## */
/** Get motor angle of OpenRAVE model*/
float
OpenRAVEMotor::getAngle() const
{
  return __angle;
}

/** Get motor angle of real robot */
float
OpenRAVEMotor::getAngleRobot() const
{
  return angleOR2Robot(__angle);
}



/* ########## setter ########## */
/** Set motor angle
 * @param angle angle of motor in OpenRAVE
 */
void
OpenRAVEMotor::setAngle(float angle)
{
  __angle = angle;
}

/** Set motor angle
 * @param angle angle of robot's motor
 */
void
OpenRAVEMotor::setAngleRobot(float angle)
{
  __angle = angleRobot2OR(angle);
}

} // end namespace fawkes