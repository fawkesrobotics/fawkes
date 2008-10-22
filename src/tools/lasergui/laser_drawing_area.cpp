
/***************************************************************************
 *  laser_drawing_area.cpp - Laser drawing area derived from Gtk::DrawingArea
 *
 *  Created: Thu Oct 09 18:20:21 2008
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

#include "laser_drawing_area.h"
#include <interfaces/laser360.h>
#include <utils/math/angle.h>
#include <gui_utils/robot/drawer.h>

using namespace fawkes;

/** @class LaserDrawingArea "laser_drawing_area.h"
 * Laser drawing area.
 * Derived version of Gtk::DrawingArea that renders values of a laser interface.
 * @author Tim Niemueller
 */

/** Constructor.
 * Special ctor to be used with Glade's get_widget_derived().
 * @param cobject Gtk C object
 * @param refxml Glade's XML reference
 */
LaserDrawingArea::LaserDrawingArea(BaseObjectType* cobject,
				   const Glib::RefPtr<Gnome::Glade::Xml>& refxml)
  : Gtk::DrawingArea(cobject)
{
  __draw_mode = MODE_LINES;
  __zoom_factor = 50;
  __laser_if = NULL;
  __robot_drawer = NULL;
  __resolution = 1;
  __rotation = 0;

  add_events(Gdk::SCROLL_MASK);

#ifndef GLIBMM_DEFAULT_SIGNAL_HANDLERS_ENABLED
  signal_expose_event().connect(sigc::mem_fun(*this, &LaserDrawingArea::on_expose_event));
#endif
}

/** Constructor. */
LaserDrawingArea::LaserDrawingArea()
{
  __draw_mode = MODE_LINES;
  __zoom_factor = 50;
  __laser_if = NULL;
  __robot_drawer = NULL;
  __resolution = 1;
  __rotation = 0;

  add_events(Gdk::SCROLL_MASK);
#ifndef GLIBMM_DEFAULT_SIGNAL_HANDLERS_ENABLED
  signal_expose_event().connect(sigc::mem_fun(*this, &LaserDrawingArea::on_expose_event));
#endif
}

/** Set laser interface.
 * @param laser_if laser interface
 */
void
LaserDrawingArea::set_laser_if(Laser360Interface *laser_if)
{
  __laser_if = laser_if;
}

/** Set robot drawer.
 * @param robot_drawer new robot drawer to use
 */
void
LaserDrawingArea::set_robot_drawer(fawkes::CairoRobotDrawer *robot_drawer)
{
  __robot_drawer = robot_drawer;
}

/** Set resolution.
 * Every n'th beam will be drawn where n is the resolution.
 * @param resolution new resolution
 */
void
LaserDrawingArea::set_resolution(unsigned int resolution)
{
  __resolution = resolution;
}


/** Set the drawing mode.
 * @param mode the new drawing mode
 */
void
LaserDrawingArea::set_draw_mode(draw_mode_t mode)
{
  __draw_mode = mode;
  queue_draw();
}

/** Zoom in.
 * Increases zoom factor by 20, no upper limit.
 */
void
LaserDrawingArea::zoom_in()
{
  __zoom_factor += 20;
  queue_draw();
}

/** Zoom out.
 * Decreases zoom factor with a minimum of 1.
 */
void
LaserDrawingArea::zoom_out()
{
  if ( __zoom_factor > 20 ) {
    __zoom_factor -= 20;
  } else {
    __zoom_factor = 1;
  }
  queue_draw();
}


/** Set rotation.
 * @param rot_rad rotation angle in rad
 */
void
LaserDrawingArea::set_rotation(float rot_rad)
{
  __rotation = rot_rad;
}


/** Draw robot.
 * @param window Gdk window
 * @param cr Cairo context to draw to. It is assumed that possible transformations
 * have been setup before.
 */
void
LaserDrawingArea::draw_scalebox(Glib::RefPtr<Gdk::Window> &window,
				Cairo::RefPtr<Cairo::Context> &cr)
{
  cr->save();
  cr->set_source_rgba(0, 0, 0.8, 0.2);
  cr->rectangle(-3, -2, 6, 4);
  cr->stroke();
  cr->restore();
}


/** Expose event handler.
 * @param event event info structure.
 * @return signal return value
 */
bool
LaserDrawingArea::on_expose_event(GdkEventExpose* event)
{
  // static Clock *clock = Clock::instance();
  // static Time last(clock);
  // static unsigned int updates = 0;
  // Time now(clock);
  // now.stamp();
  // if ( now - &last > 1 ) {
  //   printf("Updates/s: %u\n", updates);
  //   last.stamp();
  //   updates = 0;
  // } else {
  //   ++updates;
  // }

  // This is where we draw on the window
  Glib::RefPtr<Gdk::Window> window = get_window();
  if(window) {
    Gtk::Allocation allocation = get_allocation();
    const int width = allocation.get_width();
    const int height = allocation.get_height();
    
    // coordinates for the center of the window
    int xc, yc;
    xc = width / 2;
    yc = height / 2;
    
    Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();
    cr->set_line_width(1.0);

    // clip to the area indicated by the expose event so that we only redraw
    // the portion of the window that needs to be redrawn
    cr->rectangle(event->area.x, event->area.y,
		  event->area.width, event->area.height);
    cr->set_source_rgb(1, 1, 1);
    cr->fill_preserve();
    cr->clip();
    cr->set_source_rgb(0, 0, 0);
    cr->translate(xc, yc);
  
    cr->save();
    if ( __laser_if == NULL ) {
      Cairo::TextExtents te;
      std::string t = "Not connected to BlackBoard";
      cr->set_source_rgb(1, 0, 0);
      cr->set_font_size(20);
      cr->get_text_extents(t, te);
      cr->move_to(- te.width / 2, -te.height / 2);
      cr->show_text(t);
    } else if ( ! __laser_if->has_writer() ) {
      Cairo::TextExtents te;
      std::string t = "No writer for laser interface";
      cr->set_source_rgb(1, 0, 0);
      cr->set_font_size(20);
      cr->get_text_extents(t, te);
      cr->move_to(- te.width / 2, -te.height / 2);
      cr->show_text(t);
    } else {
      __laser_if->read();
      float *distances = __laser_if->distances();
      size_t nd = __laser_if->maxlenof_distances();

      cr->scale(__zoom_factor, __zoom_factor);
      cr->rotate(__rotation);
      cr->set_line_width(1. / __zoom_factor);

      draw_scalebox(window, cr);

      if ( __draw_mode == MODE_LINES ) {
	for (size_t i = 0; i < nd; i += __resolution) {
	  if ( distances[i] == 0 )  continue;
	  float anglerad = deg2rad(i);
	  cr->move_to(0, 0);
	  cr->line_to(distances[i] *  sin(anglerad),
		      distances[i] * -cos(anglerad));
	}
	cr->stroke();
      } else if ( __draw_mode == MODE_POINTS ) {
	float radius = 4 / __zoom_factor;
	for (size_t i = 0; i < nd; i += __resolution) {
	  if ( distances[i] == 0 )  continue;
	  float anglerad = deg2rad(i);
	  float x = distances[i] *  sin(anglerad);
	  float y = distances[i] * -cos(anglerad);
	  // circles replaced by rectangles, they are a *lot* faster
	  //cr->move_to(x, y);
	  //cr->arc(x, y, radius, 0, 2*M_PI);
	  cr->rectangle(x, y, radius, radius);
	}
	cr->fill_preserve();
	cr->stroke();
      } else {
	cr->move_to(0, - distances[0]);
	for (size_t i = __resolution; i <= nd + __resolution; i += __resolution) {
	  if ( distances[i] == 0 )  continue;
	  float anglerad    = deg2rad(i % 360);
	  cr->line_to(distances[i % 360] *  sin(anglerad),
		      distances[i % 360] * -cos(anglerad));
	}
	cr->stroke();
      }
      if (__robot_drawer)  __robot_drawer->draw_robot(window, cr);
    }
    cr->restore();
  }

  return true;
}

/** Scroll event handler.
 * @param event event structure
 * @return signal return value
 */
bool
LaserDrawingArea::on_scroll_event(GdkEventScroll *event)
{
  if (event->direction == GDK_SCROLL_UP) {
    zoom_in();
  } else if (event->direction == GDK_SCROLL_DOWN) {
    zoom_out();
  }
  return true;
}
