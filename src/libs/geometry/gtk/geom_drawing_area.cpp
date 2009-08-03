
/***************************************************************************
 *  geom_drawing_area.cpp - A Gtk::DrawingArea for objects of the Fawkes
 *  geometry library
 *
 *  Created: Wed Oct 08 18:52:10 2008
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

#include <geometry/gtk/geom_drawing_area.h>
#include <geometry/hom_point.h>
#include <geometry/gtk/hom_point_drawer.h>
#include <geometry/hom_vector.h>
#include <geometry/gtk/hom_vector_drawer.h>
#include <geometry/line_segment.h>
#include <geometry/gtk/line_segment_drawer.h>
#include <geometry/bezier.h>
#include <geometry/gtk/spline_drawer.h>
#include <geometry/spline.h>
#include <geometry/gtk/bezier_drawer.h>
#include <geometry/gtk/drawing_manipulator.h>
#include <cmath>

using namespace std;

namespace fawkes{

/** @class fawkes::GeomDrawingArea <geometry/gtk/geom_drawing_area.h>
 * A Gtk::DrawingArea that allows to easily display drawable objects
 * of the geometry library.
 * @author Daniel Beck
 */

/** @var fawkes::GeomDrawingArea::m_drawers
 * A list of drawers for objects that have been requested to be drawn.
 */

/** @var fawkes::GeomDrawingArea::m_max_x
 * Right boundary of the drawing area.
 */

/** @var fawkes::GeomDrawingArea::m_max_y
 * Top boundary of the drawing area.
 */

/** @var fawkes::GeomDrawingArea::m_min_x
 * Left boundary of the drawing area.
 */

/** @var fawkes::GeomDrawingArea::m_min_y
 * Bottom boundary of the drawing area.
 */

/** Constructor.
 * @param max_x top right corner
 * @param max_y top right corner
 * @param min_x bottom left corner
 * @param min_y bottom left corner
 */
GeomDrawingArea::GeomDrawingArea( float max_x, 
				  float max_y,
				  float min_x,
				  float min_y )
  : m_max_x(max_x),
    m_max_y(max_y),
    m_min_x(min_x),
    m_min_y(min_y)
{
  m_cur_drawing_manipulator = NULL;
}

/** Constructor.
 * @param cobject pointer to the base object
 * @param ref_xml Glade XML file
 */
GeomDrawingArea::GeomDrawingArea( BaseObjectType* cobject,
				  const Glib::RefPtr<Gnome::Glade::Xml>& ref_xml )
  : Gtk::DrawingArea(cobject)
{
  m_max_x =  5.0;
  m_max_y =  5.0;
  m_min_x = -5.0;
  m_min_y = -5.0;

  m_cur_drawing_manipulator = NULL;
}

/** Destructor. */
GeomDrawingArea::~GeomDrawingArea()
{
  clear();
}

/** Clear the drawing area. */
void
GeomDrawingArea::clear()
{
  for ( vector<GeomDrawer*>::iterator iter = m_drawers.begin();
	iter != m_drawers.end();
	++iter )
    {
      delete *iter;
    }

  m_drawers.clear();

  m_cur_drawing_manipulator = NULL;
}

/** <<-operator for HomPoint objects
 * @param p a HomPoint object
 * @return a reference to the drawing area
 */
GeomDrawingArea&
GeomDrawingArea::operator<<(fawkes::HomPoint& p)
{
  HomPointDrawer* d = new HomPointDrawer(p);

  if (m_cur_drawing_manipulator)
    { d->set_point_size( m_cur_drawing_manipulator->get_point_size() ); }

  m_drawers.push_back(d);

  return *this;
}

/** <<-operator for HomPoint objects
 * @param p a HomPoint object
 * @return a reference to the drawing area
 */
GeomDrawingArea&
GeomDrawingArea::operator<<(const fawkes::HomPoint& p)
{
  HomPointDrawer* d = new HomPointDrawer(p);

  if (m_cur_drawing_manipulator)
    { d->set_point_size( m_cur_drawing_manipulator->get_point_size() ); }

  m_drawers.push_back(d);

  return *this;
}

/** <<-operator for HomVector objects
 * @param vp a pair constisting of the vector and the offset
 * @return a reference to the drawing area
 */
GeomDrawingArea&
GeomDrawingArea::operator<<(std::pair<HomVector, HomPoint> vp)
{
  const HomVector& v = vp.first;
  const HomPoint& offset = vp.second;
  HomVectorDrawer* d = new HomVectorDrawer(v, offset);
  m_drawers.push_back(d);

  return *this;
}

/** <<-operator for LineSegments objects
 * @param l a LineSegment object
 * @return a reference to the drawing area
 */
GeomDrawingArea&
GeomDrawingArea::operator<<(fawkes::LineSegment& l)
{
  LineSegmentDrawer* d = new LineSegmentDrawer(l);
  m_drawers.push_back(d);

  return *this;
}

/** <<-operator for Bezier objects.
 * @param b a Bezier object
 * @return a reference to the drawing area
 */
GeomDrawingArea&
GeomDrawingArea::operator<<(fawkes::Bezier& b)
{
  BezierDrawer* d = new BezierDrawer(b);
  m_drawers.push_back(d);

  return *this;
}

/** <<-operator for Spline objects.
 * @param s a Spline object
 * @return a reference to the drawing area
 */
GeomDrawingArea&
GeomDrawingArea::operator<<(fawkes::Spline& s)
{
  SplineDrawer* d = new SplineDrawer(s);
  m_drawers.push_back(d);

  return *this;
}

/** <<-operator for Spline objects.
 * @param s a Spline object
 * @return a reference to the drawing area
 */
GeomDrawingArea&
GeomDrawingArea::operator<<(const fawkes::Spline& s)
{
  SplineDrawer* d = new SplineDrawer(s);
  m_drawers.push_back(d);

  return *this;
}

/** <<-operator for DrawingManipulator objects.
 * Note: the drawing area takes over the ownwership of the manipulator.
 * @param m a DrawingManipulator object
 * @return a reference to the drawing area
 */
GeomDrawingArea&
GeomDrawingArea::operator<<(fawkes::DrawingManipulator* m)
{
  if (m_cur_drawing_manipulator)
    { m->integrate(m_cur_drawing_manipulator); }

  m_cur_drawing_manipulator = m;
  m_drawers.push_back(m);

  return *this;
}

/** Signal handler for the expose event.
 * @param event the event
 * @return true if event has been handled
 */
bool
GeomDrawingArea::on_expose_event(GdkEventExpose* event)
{
  Glib::RefPtr<Gdk::Window> window = get_window();
  if (window)
    {
      Gtk::Allocation allocation = get_allocation();
      m_window_width = allocation.get_width();
      m_window_height = allocation.get_height();
      
      Cairo::RefPtr<Cairo::Context> context = window->create_cairo_context();
      
      if (event)
	{
	  context->rectangle( event->area.x, event->area.y,
			      event->area.width, event->area.height );
	  context->clip();
	}
      
      float unit_width  = fabs(m_max_x) + fabs(m_min_x);
      float unit_height = fabs(m_max_y) + fabs(m_min_y);
      if ( (m_window_width / unit_width) <= (m_window_height / unit_height) )
	{ m_unit = m_window_width / unit_width; }
      else
	{ m_unit = m_window_height / unit_height; }
      
      pre_draw(context);
      
      for ( vector<GeomDrawer*>::iterator iter = m_drawers.begin();
	    iter != m_drawers.end();
	    ++iter )
	{
	  GeomDrawer* d = *iter;
	  d->draw(context);
	}

      post_draw(context);

      context->stroke();
    }

  return true;
}

/** Convert the given window coordinates into the frame of the drawing area.
 * @param window_x the window coordinate
 * @param window_y the window coordinate
 * @param drawing_x the drawing coordinate
 * @param drawing_y the drawing coordinate
 */
void
GeomDrawingArea::to_drawing_coords( int window_x, int window_y,
				    float& drawing_x, float& drawing_y )
{
  float unit_width  = fabs(m_max_x) + fabs(m_min_x);
  
  float pixel_per_unit = m_window_width / unit_width;

  drawing_x =   window_x / pixel_per_unit + m_min_x;
  drawing_y = -(window_y / pixel_per_unit + m_min_y);
}

/** This method is called by the expose signal handler before the draw
 * routines of the registered drawers are called.
 * Derived classes might want to change this to add static drawing
 * elements or to change the viewing matrix.
 * @param context the drawing context
 */
void
GeomDrawingArea::pre_draw(Cairo::RefPtr<Cairo::Context>& context)
{
  context->translate( m_window_width / 2.0, m_window_height / 2.0 );
  context->scale(m_unit, -m_unit);
}

/** This method is called by the expose signal handler after the
 * draw routines of the registered drawers are called.
 * Derived classes might want to change this to add static drawing
 * elements.
 * @param context the drawing context
 */
void
GeomDrawingArea::post_draw(Cairo::RefPtr<Cairo::Context>& context)
{
}

} // end namespace fawkes
