
/***************************************************************************
 *  geom_drawing_area.h - A Gtk::DrawingArea for objects of the Fawkes
 *  geometry library
 *
 *  Created: Wed Oct 08 18:35:19 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __GEOMETRY_GEOM_DRAWING_AREA_H_
#define __GEOMETRY_GEOM_DRAWING_AREA_H_

#include <gtkmm.h>
#include <libglademm/xml.h>

#include <vector>

namespace fawkes{
class GeomDrawer;
class HomPoint;
class HomVector;
class LineSegment;
class Bezier;
class Spline;
class DrawingManipulator;

class GeomDrawingArea : public Gtk::DrawingArea
{
 public:
  GeomDrawingArea( float max_x =  5.0, 
		   float max_y =  5.0,
		   float min_x = -5.0,
		   float min_y = -5.0 );
  GeomDrawingArea(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml>& ref_xml);
  virtual ~GeomDrawingArea();

  void clear();

  GeomDrawingArea& operator<<(fawkes::HomPoint& p);
  GeomDrawingArea& operator<<(const fawkes::HomPoint& p);
  GeomDrawingArea& operator<<(std::pair<HomVector, HomPoint> v);
  GeomDrawingArea& operator<<(fawkes::LineSegment& l);
  GeomDrawingArea& operator<<(fawkes::Bezier& b);
  GeomDrawingArea& operator<<(fawkes::Spline& s);
  GeomDrawingArea& operator<<(const fawkes::Spline& s);
  GeomDrawingArea& operator<<(fawkes::DrawingManipulator* m);

  virtual void to_drawing_coords(int window_x, int window_y, float& drawing_x, float& drawing_y);

 protected:
  virtual void pre_draw(Cairo::RefPtr<Cairo::Context>& context);
  virtual void post_draw(Cairo::RefPtr<Cairo::Context>& context);

 private:
  virtual bool on_expose_event(GdkEventExpose* event);

  std::vector<fawkes::GeomDrawer*> m_drawers;
  fawkes::DrawingManipulator*      m_cur_drawing_manipulator;

  float m_max_x;
  float m_max_y;
  float m_min_x;
  float m_min_y;

  float m_unit;

  int m_window_width;
  int m_window_height;
};

} // end namespace fawkes

#endif /* __GEOMETRY_GEOM_DRAWING_AREA_H_ */
