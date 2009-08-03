
/***************************************************************************
 *  bezier_drawer.cpp - Drawer for the Bezier class
 *
 *  Created: Thu Oct 09 15:08:38 2008
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

#include <geometry/gtk/bezier_drawer.h>
#include <geometry/bezier.h>
#include <geometry/hom_point.h>

/** @class fawkes::BezierDrawer <geometry/gtk/bezier_drawer.h>
 * Drawer for Bezier objects.
 * @author Daniel Beck
 */

using namespace std;

namespace fawkes {

/** Constructor.
 * @param b the Bezier to draw
 */
BezierDrawer::BezierDrawer(Bezier& b)
{
  m_bezier = &b;
}

/** Destructor. */
BezierDrawer::~BezierDrawer()
{
}

void
BezierDrawer::draw(Cairo::RefPtr<Cairo::Context>& context)
{
  vector<HomPoint> points = m_bezier->approximate();

  vector<HomPoint>::const_iterator prev = points.begin();
  vector<HomPoint>::const_iterator cur  = prev;
  ++cur;
  
  while ( cur != points.end() )
    {
      context->save();
      context->move_to( prev->x(), prev->y() );
      context->line_to( cur->x(), cur->y() );
      context->restore();

      ++prev;
      ++cur;
    }

  context->stroke();
}

} // end namespace fawkes
