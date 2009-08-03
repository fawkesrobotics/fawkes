
/***************************************************************************
 *  drawing_manipulator.cpp - Manipulates things like point size, line
 *  width, etc.
 *
 *  Created: Fri Oct 10 18:20:02 2008
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

#include <geometry/gtk/drawing_manipulator.h>

/** @class fawkes::DrawingManipulator <geometry/gtk/drawing_manipulator.h>
 * Allows to control some aspects of the rendering of objects.
 * @author Daniel Beck
 */

/** @enum fawkes::DrawingManipulator::Color
 * Some pre-defined colors
 */

namespace fawkes {

/** Constructor. */
DrawingManipulator::DrawingManipulator()
{
  m_line_width = -1.0;
  m_color_r = -1.0;
  m_color_g = -1.0;
  m_color_b = -1.0;

  m_line_width_set = false;
  m_point_size_set = false;
  m_color_set = false;
}

/** Desctructor. */
DrawingManipulator::~DrawingManipulator()
{
}

/** Integrates the parameters of another manipulator.
 * If a certain field in this manipulator is not set it is assigned
 * the respective value from the specified manipualator.
 * @param m the manipulator to integrate
 */
void
DrawingManipulator::integrate(const DrawingManipulator* m)
{
  if ( !m_line_width_set && m->m_line_width_set )
    { m_line_width = m->m_line_width; }

  if ( !m_point_size_set && m->m_point_size_set)
    { m_point_size = m->m_point_size; }

  if ( !m_color_set && m->m_color_set)
    {
      m_color_r = m->m_color_r;
      m_color_g = m->m_color_g;
      m_color_b = m->m_color_b;
    }
}

/** Set the line width. 
 * @param w the line width
 */
void
DrawingManipulator::set_line_width(float w)
{
  if (0 > w)
    { return; }

  m_line_width = w;
  m_line_width_set = true;
}

/** Get the line width.
 * @return the line width
 */
float
DrawingManipulator::get_line_width() const
{
  return m_line_width;
}

/** Set the point size.
 * @param s the point size
 */
void
DrawingManipulator::set_point_size(float s)
{
  if (0 > s)
    { return; }

  m_point_size = s;
  m_point_size_set = true;
}

/** Get the point size.
 * @return the point size
 */
float
DrawingManipulator::get_point_size() const
{
  return m_point_size;
}

/** Set the color.
 * @param c the color
 */
void
DrawingManipulator::set_color(Color c)
{
  switch (c)
    {
    case BLACK:
      m_color_r = 0.0;
      m_color_g = 0.0;
      m_color_b = 0.0;
      m_color_set = true;
      break;

    case WHITE:
      m_color_r = 1.0;
      m_color_g = 1.0;
      m_color_b = 1.0;
      m_color_set = true;
      break;

    case RED:
      m_color_r = 1.0;
      m_color_g = 0.0;
      m_color_b = 0.0;
      m_color_set = true;
      break;

    case GREEN:
      m_color_r = 0.0;
      m_color_g = 1.0;
      m_color_b = 0.0;
      m_color_set = true;
      break;

    case BLUE:
      m_color_r = 0.0;
      m_color_g = 0.0;
      m_color_b = 1.0;
      m_color_set = true;
      break;

    default:
      break;
    }
}

/** Set the color specified in RGB.
 * @param r the R value of the color
 * @param g the G value of the color
 * @param b the B value of the color
 */
void
DrawingManipulator::set_color(float r, float g, float b)
{
  if ( 0 > r || 1 < r || 0 > g || 1 < g || 0 > b || 1 < b )
    { return; }

  m_color_r = r;
  m_color_g = g;
  m_color_b = b;

  m_color_set = true;
}

/** Get the color.
 * @param r reference to a variable where the R value of the current color is written to
 * @param g reference to a variable where the G value of the current color is written to
 * @param b reference to a variable where the B value of the current color is written to
 */
void
DrawingManipulator::get_color(float& r, float& g, float& b) const
{
  r = m_color_r;
  g = m_color_g;
  b = m_color_b;
}

void
DrawingManipulator::draw(Cairo::RefPtr<Cairo::Context>& context)
{
  if (m_line_width_set)
    { context->set_line_width(m_line_width); }

  if (m_color_set)
    { context->set_source_rgb(m_color_r, m_color_g, m_color_b); }
}

/** Creates a drawing manipulator which sets the given line width.
 * @param w the line width
 * @return pointer to the newly created drawing manipulator
 */
DrawingManipulator* set_line_width(float w)
{
  DrawingManipulator* m = new DrawingManipulator();
  m->set_line_width(w);
  return m;
}

/** Creates a drawing manipulator which sets the given color.
 * @param r the R value of the color
 * @param g the G value of the color
 * @param b the B value of the color
 * @return pointer to the newly created drawing manipulator
 */
DrawingManipulator* set_color(float r, float g, float b)
{
  DrawingManipulator* m = new DrawingManipulator();
  m->set_color(r, g, b);
  return m;
}

/** Creates a drawing manipulator which sets the given color.
 * @param c one of the colors defined in the DrawingManipulator class
 * @return pointer to the newly created drawing manipulator
 */
DrawingManipulator* set_color(DrawingManipulator::Color c)
{
  DrawingManipulator* m = new DrawingManipulator();
  m->set_color(c);
  return m;
}

/** Creates a drawing manipulator which sets the given point size.
 * @param s the point size
 * @return pointer to the newly created drawing manipulator
 */
DrawingManipulator* set_point_size(float s)
{
  DrawingManipulator* m = new DrawingManipulator();
  m->set_point_size(s);
  return m;
}
  
} // end namespace fawkes
