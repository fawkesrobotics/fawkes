
/***************************************************************************
 *  motor.h - Fawkes to OpenRAVE Motor Data
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

#ifndef __OPENRAVE_MOTOR_H_
#define __OPENRAVE_MOTOR_H_

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenRAVEMotor
{
 public:
  OpenRAVEMotor(unsigned int number);
  virtual ~OpenRAVEMotor();

  virtual float getAngle() const; // angle in OpenRAVE
  virtual float	getAngleRobot() const; // real robot's angle

  virtual void setAngle(float angle);
  virtual void setAngleRobot(float angle);

 protected:
  virtual float angleOR2Robot(float angle) const;
  virtual float angleRobot2OR(float angle) const;

  unsigned int  __no; // motor number
  float		__angle; //radian
};

} // end of namespace fawkes

#endif