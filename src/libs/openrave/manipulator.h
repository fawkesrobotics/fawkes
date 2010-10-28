
/***************************************************************************
 *  manipulator.h - Fawkes to OpenRAVE Manipulator Data
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

#ifndef __OPENRAVE_MANIPULATOR_H_
#define __OPENRAVE_MANIPULATOR_H_

#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

typedef struct {
  unsigned int	no;       /**< motor number in OpenRAVE*/
  unsigned int  noDevice;  /**< motor number of real device */
  float		angle;	  /**< radian angle */
} motor_t;

class OpenRAVEManipulator
{
 public:
  OpenRAVEManipulator(unsigned int count, unsigned int countDevice);
  virtual ~OpenRAVEManipulator();

  virtual void addMotor(unsigned int number, unsigned int numberDevice);

  virtual std::vector<float> anglesOR2Device(std::vector<float>& from) const;
  virtual void getAngles(std::vector<float>& v) const; // angles of OpenRAVE model
  virtual void getAnglesDevice(std::vector<float>& v) const; // angles of real device

  virtual void setAngles(std::vector<float>& angles);
  virtual void setAnglesDevice(std::vector<float>& angles);


 protected:
  virtual float angleOR2Device(unsigned int number, float angle) const;
  virtual float angleDevice2OR(unsigned int number, float angle) const;

  std::vector<motor_t>  __motors;
  unsigned int          __cnt;
  unsigned int          __cntDevice;
};

} // end of namespace fawkes

#endif