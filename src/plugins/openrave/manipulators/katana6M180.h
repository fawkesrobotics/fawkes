
/***************************************************************************
 *  katana6M180.h - Fawkes to OpenRAVE Katana6M180 Manipulator Data
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

#ifndef __PLUGINS_OPENRAVE_MANIPULATORS_KATANA6M180_H_
#define __PLUGINS_OPENRAVE_MANIPULATORS_KATANA6M180_H_

#include "../manipulator.h"

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenRaveManipulatorKatana6M180 : public OpenRaveManipulator
{
 public:
  OpenRaveManipulatorKatana6M180(unsigned int count, unsigned int countDevice);
  virtual ~OpenRaveManipulatorKatana6M180();

  virtual OpenRaveManipulatorPtr copy();

 private:
  virtual float angle_OR_to_device(unsigned int number, float angle) const;
  virtual float angle_device_to_OR(unsigned int number, float angle) const;
};

} // end of namespace fawkes

#endif
