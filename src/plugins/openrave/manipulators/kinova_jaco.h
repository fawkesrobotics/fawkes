
/***************************************************************************
 *  kinova_jaco.h - Fawkes to OpenRAVE Kinova Jaco Manipulator Data
 *
 *  Created: Sun Jun 23 21:12:52 2013
 *  Copyright  2013  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#ifndef __PLUGINS_OPENRAVE_MANIPULATORS_KINOVA_JACO_H_
#define __PLUGINS_OPENRAVE_MANIPULATORS_KINOVA_JACO_H_

#include "../manipulator.h"

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenRaveManipulatorKinovaJaco : public OpenRaveManipulator
{
 public:
  OpenRaveManipulatorKinovaJaco(unsigned int count, unsigned int countDevice);
  virtual ~OpenRaveManipulatorKinovaJaco();


 private:
  virtual float angle_OR_to_device(unsigned int number, float angle) const;
  virtual float angle_device_to_OR(unsigned int number, float angle) const;
};

} // end of namespace fawkes

#endif