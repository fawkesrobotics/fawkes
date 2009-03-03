
/***************************************************************************
 *  line_segment.cpp - A line segment
 *
 *  Created: Thu Oct 02 17:05:52 2008
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
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

#include <geometry/line_segment.h>

namespace fawkes {

/** @class fawkes::LineSegment <geometry/line_segment.h>
 * A line segment.
 * @author Daniel Beck
 */

/** Constructor.
 * @param a the starting point of the line segment
 * @param b the endpoint of of the line segment
 */
LineSegment::LineSegment(const HomPoint& a, const HomPoint& b)
  : m_p1(a),
    m_p2(b)
{
  register_primitives();
}

/** Constructor.
 * @param p the starting point of the line segment
 * @param v a vector defining orientation and length of the line
 * segment
 */
LineSegment::LineSegment(const HomPoint& p, const HomVector& v)
  : m_p1(p),
    m_p2(p+v)
{
  register_primitives();
}

/** Copy constructor.
 * @param l another line segment
 */
LineSegment::LineSegment(const LineSegment& l)
  : m_p1(l.m_p1),
    m_p2(l.m_p2)
{
  clear_primitives();
  register_primitives();
}

/** Destructor. */
LineSegment::~LineSegment()
{
}

/** Get the length of the line segment.
 * @return the length of the line segment
 */
float
LineSegment::length() const
{
  HomVector v;
  v = m_p2 - m_p1;
  return v.length();
}

/** Get the starting point.
 * @return the starting point
 */
const HomPoint&
LineSegment::p1() const
{
  return m_p1;
}

/** Get the endpoint.
 * @return the endpoint
 */
const HomPoint&
LineSegment::p2() const
{
  return m_p2;
}

void
LineSegment::register_primitives()
{
  add_primitive(&m_p1);
  add_primitive(&m_p2);
}

void
LineSegment::post_transform()
{
}

std::ostream&
LineSegment::print(std::ostream& stream) const
{
  stream << "P1: " << m_p1 << " P2: " << m_p2;
  return stream;
}

} // end namespace fawkes
