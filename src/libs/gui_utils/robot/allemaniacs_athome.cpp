
/***************************************************************************
 *  allemaniacs_athome.cpp - CairoRobotDrawer for AllemaniACs AtHome robot
 *
 *  Created: Fri Oct 10 10:37:09 2008
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

#include <gui_utils/robot/allemaniacs_athome.h>

namespace fawkes {

/** @class AllemaniACsAtHomeCairoRobotDrawer <gui_utils/robot/allemaniacs_athome.h>
 * Draw AllemaniACs AtHome robot.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param laser_at_center if true the laser of the robot will be at (0,0) instead
 * of the real robot center.
 */
AllemaniACsAtHomeCairoRobotDrawer::AllemaniACsAtHomeCairoRobotDrawer(bool laser_at_center)
{
  __laser_at_center = laser_at_center;
}

void
AllemaniACsAtHomeCairoRobotDrawer::draw_robot(Glib::RefPtr<Gdk::Window> &window,
					      const Cairo::RefPtr<Cairo::Context> &cr)
{
  cr->save();

  if ( __laser_at_center ) {
    cr->translate(0, -0.12);
  }

  // body
  cr->set_source_rgba(0.6, 0.6, 0.6, 0.6);
  cr->rectangle(-0.2, -0.2, 0.4, 0.38);
  cr->fill_preserve();
  cr->set_source_rgba(0.4, 0.4, 0.4, 0.6);
  cr->stroke();

  // yellow arrow
  cr->move_to(0, -0.175);
  cr->line_to(0.18, 0.17);
  cr->line_to(-0.18, 0.17);
  cr->line_to(0, -0.175);
  cr->set_source_rgba(1, 1, 0, 0.5);
  cr->fill_preserve();
  cr->stroke();

  // poles
  cr->set_source_rgba(0.4, 0.4, 0.4, 0.6);
  cr->arc(-0.19, -0.19, 0.005, 0, 2*M_PI);
  cr->fill_preserve(); cr->stroke();
  cr->arc(-0.19, +0.17, 0.005, 0, 2*M_PI);
  cr->fill_preserve(); cr->stroke();
  cr->arc(+0.19, +0.17, 0.005, 0, 2*M_PI);
  cr->fill_preserve(); cr->stroke();
  cr->arc(+0.19, -0.19, 0.005, 0, 2*M_PI);
  cr->fill_preserve(); cr->stroke();

  // laser
  cr->set_source_rgba(0.4, 0.4, 0.4, 0.2);
  cr->arc(0, 0.12, 0.03, 0, 2*M_PI);
  cr->fill_preserve(); cr->stroke();

  cr->restore();
}


}
