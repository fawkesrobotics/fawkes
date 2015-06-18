
/***************************************************************************
 *  laser_drawing_area.h - Laser drawing area derived from Gtk::DrawingArea
 *
 *  Created: Thu Oct 09 18:19:54 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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
#include <list>
#include <interfaces/Position2DTrackInterface.h>
#include <interfaces/SwitchInterface.h>

namespace fawkes {
  class Laser360Interface;
  class Laser720Interface;
  class Laser1080Interface;
  class ObjectPositionInterface;
  class CairoRobotDrawer;
  class LegtrackerTrackinterface;
  class SwitchInterface;
  class VisualDisplay2DInterface;
}

class VisualDisplay2D;

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
		   const Glib::RefPtr<Gtk::Builder> &builder);
  ~LaserDrawingArea();

  void set_laser_ifs(const std::list<fawkes::Interface*>& laser_if);
  void reset_laser_ifs();
  void set_objpos_if(std::list<fawkes::ObjectPositionInterface*>* l_objpos_if_persons,
		     std::list<fawkes::ObjectPositionInterface*>* l_objpos_if_legs,
		     std::list<fawkes::ObjectPositionInterface*>* l_objpos_if_misc,
		     fawkes::Laser720Interface* laser_segmentation_if,
		     std::list<fawkes::Position2DTrackInterface*>* l_track_if,
		     fawkes::ObjectPositionInterface* target_if,
		     fawkes::SwitchInterface* switch_if);
  void set_line_if(fawkes::ObjectPositionInterface *line_if);
  void set_visdisp_if(fawkes::VisualDisplay2DInterface *visdisp_if);
  void set_robot_drawer(fawkes::CairoRobotDrawer *robot_drawer);
  void set_resolution(unsigned int resolution);

  void zoom_in();
  void zoom_out();

  void set_rotation(float rot_rad);
  void set_draw_mode(draw_mode_t mode);
  void set_connected(bool connected);
  
  void toggle_break_drawing();

 protected:
#if GTK_VERSION_GE(3,0)
  virtual bool on_draw(const Cairo::RefPtr<Cairo::Context> &cr);
#else
  virtual bool on_expose_event(GdkEventExpose *event);
#endif
  virtual bool on_scroll_event(GdkEventScroll *event);
  virtual bool on_motion_notify_event(GdkEventMotion *event);
  virtual bool on_button_press_event(GdkEventButton *event);

  void draw_beams(const fawkes::Interface* itf,
                  Glib::RefPtr<Gdk::Window> &window,
		  const Cairo::RefPtr<Cairo::Context> &cr);
  void draw_segments(const fawkes::Interface* itf,
                     Glib::RefPtr<Gdk::Window> &window,
		     const Cairo::RefPtr<Cairo::Context> &cr);
  void draw_scalebox(Glib::RefPtr<Gdk::Window> &window,
		     const Cairo::RefPtr<Cairo::Context> &cr);
  void draw_persons_legs(Glib::RefPtr<Gdk::Window> &window,
			 const Cairo::RefPtr<Cairo::Context> &cr);
  std::pair<float,float> transform_coords_from_fawkes(float p_x, float p_y);


 private:
  /// @cond INTERNALS
  struct Color {
    unsigned char r;
    unsigned char g;
    unsigned char b;
  };
  /// @endcond
  typedef std::pair<fawkes::Interface*, Color> InterfaceColorPair;
  typedef std::list<InterfaceColorPair> InterfaceColorPairList;

  bool all_laser_ifs_have_writer() const;

  InterfaceColorPairList               __laser_ifs;
  fawkes::Laser720Interface           *__laser_segmentation_if;
  fawkes::SwitchInterface             *__switch_if;
  fawkes::ObjectPositionInterface     *__target_if;

  fawkes::ObjectPositionInterface *__line_if;

  std::list<fawkes::ObjectPositionInterface*>* __l_objpos_if_persons;
  std::list<fawkes::ObjectPositionInterface*>* __l_objpos_if_legs;
  std::list<fawkes::ObjectPositionInterface*>* __l_objpos_if_misc;
  std::list<fawkes::Position2DTrackInterface*>* __l_track_if;

  bool                       __connected;
  draw_mode_t                __draw_mode;

  float                      __zoom_factor;
  unsigned int               __resolution;
  float                      __rotation;
  bool                       __break_drawing;
  bool                       __first_draw;
  double __last_mouse_x;
  double __last_mouse_y;
  double __xc;
  double __yc;

  fawkes::CairoRobotDrawer  *__robot_drawer;

  VisualDisplay2D           *__visdisp;
  fawkes::VisualDisplay2DInterface  *__visdisp_if;
};

#endif
