
/***************************************************************************
 *  line_segment_drawer.cpp - Drawer for the LineSegment class
 *
 *  Created: Thu Oct 09 14:49:20 2008
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

#include <geometry/gtk/line_segment_drawer.h>
#include <geometry/line_segment.h>

/** @class fawkes::LineSegmentDrawer <geometry/gtk/line_segment_drawer.h>
 * Drawer for LineSegment objects.
 * @author Daniel Beck
 */

namespace fawkes {

/** Constructor.
 * @param l the LineSegement to drawer
 */
LineSegmentDrawer::LineSegmentDrawer(LineSegment l)
{
  m_line_segment = new LineSegment( l );
}

/** Destructor. */
LineSegmentDrawer::~LineSegmentDrawer()
{
  delete m_line_segment;
}

void
LineSegmentDrawer::draw(Cairo::RefPtr<Cairo::Context>& context)
{
  context->save();
  context->move_to( m_line_segment->p1().x(),
		    m_line_segment->p1().y() );
  context->line_to( m_line_segment->p2().x(),
		    m_line_segment->p2().y() );
  context->stroke();
  context->restore();
}

} // end namespace fawkes
