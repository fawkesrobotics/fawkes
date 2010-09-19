
/***************************************************************************
 *  motors_katana.h - Fawkes to OpenRAVE Katana6M180 Motors Data
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

#ifndef __OPENRAVE_MOTORS_KATANA_H_
#define __OPENRAVE_MOTORS_KATANA_H_

#include "motors.h"

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenRAVEMotorsKatana : public OpenRAVEMotors
{
 public:
  OpenRAVEMotorsKatana(unsigned int count, unsigned int countDevice);
  virtual ~OpenRAVEMotorsKatana();


 private:
  virtual float angleOR2Device(unsigned int number, float angle) const;
  virtual float angleDevice2OR(unsigned int number, float angle) const;
};

} // end of namespace fawkes

#endif