
/***************************************************************************
 *  laser_drawing_area.h - Laser drawing area derived from Gtk::DrawingArea
 *
 *  Created: Thu Oct 09 18:19:54 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __TOOLS_LASERGUI_LASER_DRAWING_AREA_H_
#define __TOOLS_LASERGUI_LASER_DRAWING_AREA_H_

#include <gtkmm.h>
#include <libglademm/xml.h>

namespace fawkes {
  class Laser360Interface;
  class CairoRobotDrawer;
}

class LaserDrawingArea
  : public Gtk::DrawingArea
{
 public:
  /** Draw modes. */
  typedef enum {
    MODE_LINES,		/**< Draw beams as lines */
    MODE_POINTS,	/**< Only draw beam end points */
    MODE_HULL		/**< Draw hull of beams */
  } draw_mode_t;

  LaserDrawingArea();
  LaserDrawingArea(BaseObjectType* cobject,
		   const Glib::RefPtr<Gnome::Glade::Xml>& ref_glade);

  void set_laser_if(fawkes::Laser360Interface *laser_if);
  void set_robot_drawer(fawkes::CairoRobotDrawer *robot_drawer);
  void set_resolution(unsigned int resolution);

  void zoom_in();
  void zoom_out();

  void set_rotation(float rot_rad);
  void set_draw_mode(draw_mode_t mode);

 protected:
  virtual bool on_expose_event(GdkEventExpose* event);
  virtual bool on_scroll_event(GdkEventScroll *event);
  void draw_scalebox(Glib::RefPtr<Gdk::Window> &window,
		     Cairo::RefPtr<Cairo::Context> &cr);

 private:
  fawkes::Laser360Interface *__laser_if;

  draw_mode_t                __draw_mode;

  float                      __zoom_factor;
  unsigned int               __resolution;
  float                      __rotation;
  fawkes::CairoRobotDrawer  *__robot_drawer;
};

#endif
