
/***************************************************************************
 *  drawer.cpp - Interface for drawing a robot on a Cairo context
 *
 *  Created: Fri Oct 10 10:30:05 2008
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

#include <gui_utils/robot/drawer.h>

namespace fawkes {

/** @class CairoRobotDrawer <gui_utils/robot/drawer.h>
 * Robot drawing interface.
 * This interface provides a generic way to draw robots in different applications.
 * @author Tim Niemueller
 *
 * @fn void CairoRobotDrawer::draw_robot(Glib::RefPtr<Gdk::Window> &window, const Cairo::RefPtr<Cairo::Context> &cr) = 0
 * Draw robot.
 * This method must be implemented to do the actual drawing. The cairo context can
 * be assumed to be translated so that the robot's center is at (0,0) and all
 * values for coordinates are given in meters.
 * @param window Gdk window the Cairo context is associated to
 * @param cr Cairo context
 */

CairoRobotDrawer::~CairoRobotDrawer()
{
}

}
