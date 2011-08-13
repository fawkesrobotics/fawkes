
/***************************************************************************
 *  allemaniacs_athome.h - CairoRobotDrawer for AllemaniACs AtHome robot
 *
 *  Created: Fri Oct 10 10:35:52 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __LIBS_GUI_UTILS_ROBOT_ALLEMANIACS_ATHOME_H_
#define __LIBS_GUI_UTILS_ROBOT_ALLEMANIACS_ATHOME_H_

#include <gui_utils/robot/drawer.h>

namespace fawkes {

class AllemaniACsAtHomeCairoRobotDrawer : public CairoRobotDrawer
{
 public:
  AllemaniACsAtHomeCairoRobotDrawer(bool laser_at_center = false);

  virtual void draw_robot(Glib::RefPtr<Gdk::Window> &window,
			  const Cairo::RefPtr<Cairo::Context> &cr);

 private:
  bool __laser_at_center;
};

}

#endif
