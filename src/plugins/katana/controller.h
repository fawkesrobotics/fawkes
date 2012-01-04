
/***************************************************************************
 *  controller.h - Controller class for katana arm
 *
 *  Created: Tue Jan 03 11:40:31 2012
 *  Copyright  2012  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#ifndef __PLUGINS_KATANA_CONTROLLER_H_
#define __PLUGINS_KATANA_CONTROLLER_H_

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class KatanaController
{
 public:
  virtual ~KatanaController() {}

  virtual void init() = 0;

};


} // end of namespace fawkes

#endif