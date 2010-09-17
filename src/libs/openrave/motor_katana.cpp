
/***************************************************************************
 *  motor_katana.cpp - Fawkes to OpenRAVE Katana6M180 Motor Data
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

#include "motor_katana.h"
#include "motor.h"

 namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenRAVEMotor <openrave/motor_kataba.h>
 * Data container of a katan6M180 arm motor.
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param number motor number
 */
OpenRAVEMotorKatana::OpenRAVEMotorKatana(unsigned int number) :
  OpenRAVEMotor( number )
{
}

OpenRAVEMotorKatana::OpenRAVEMotorKatana(unsigned int number, float angleOffset, int encOffset, int epc, int rotDir) :
  OpenRAVEMotor( number ),
  __cfg_angleOffset( angleOffset ),
  __cfg_encoderOffset( encOffset ),
  __cfg_encodersPerCycle( epc ),
  __cfg_rotationDirection( rotDir )
{
}

/** Destructor. */
OpenRAVEMotorKatana::~OpenRAVEMotorKatana()
{
}

/* ########## various ######### */
/** Transform OpenRAVE angle to robot's angle */
float
OpenRAVEMotor::angleOR2Robot(float angle) const
{
  return angle;
}

/** Transform robot's angle to OpenRAVE angle */
float
OpenRAVEMotor::angleRobot2OR(float angle) const
{
  return angle;
}
} // end namespace fawkes