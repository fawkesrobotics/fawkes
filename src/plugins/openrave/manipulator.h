
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

#ifndef __PLUGINS_OPENRAVE_MANIPULATOR_H_
#define __PLUGINS_OPENRAVE_MANIPULATOR_H_

#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Struct containing angle of current motor, its number in OpenRAVE and
 * corresponding motor number of real devices. */
typedef struct {
  unsigned int	no;       /**< motor number in OpenRAVE*/
  unsigned int  no_device;  /**< motor number of real device */
  float		angle;	  /**< radian angle */
} motor_t;

class OpenRAVEManipulator
{
 public:
  OpenRAVEManipulator(unsigned int count, unsigned int count_device);
  virtual ~OpenRAVEManipulator();

  virtual void add_motor(unsigned int number, unsigned int number_device);

  virtual std::vector<float> angles_or_to_device(std::vector<float>& from) const;
  virtual void get_angles(std::vector<float>& v) const; // angles of OpenRAVE model
  virtual void get_angles_device(std::vector<float>& v) const; // angles of real device

  virtual void set_angles(std::vector<float>& angles);
  virtual void set_angles_device(std::vector<float>& angles);


 protected:
  virtual float angle_OR_to_device(unsigned int number, float angle) const;
  virtual float angle_device_to_OR(unsigned int number, float angle) const;

  std::vector<motor_t>  __motors;       /**< vector of motors */
  unsigned int          __cnt;
  unsigned int          __cnt_device;
};

} // end of namespace fawkes

#endif