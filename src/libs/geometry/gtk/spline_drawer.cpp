
/***************************************************************************
 *  spline_drawer.cpp - Drawer for the Spline class
 *
 *  Created: Fri Oct 10 14:00:22 2008
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

#include <geometry/gtk/spline_drawer.h>
#include <geometry/gtk/bezier_drawer.h>
#include <geometry/spline.h>

/** @class fawkes::SplineDrawer <geometry/gtk/spline_drawer.h>
 * Drawer for Spline objects.
 * @author Daniel Beck
 */

using namespace std;

namespace fawkes {

/** Constructor.
 * This constructor does not copy the Spline object but keeps a
 * pointer to the specified Spline object. Consequently, you have to
 * make sure that the object is not deleted before it is drawn. If you
 * cannot ensure this use the constructor that is given a const
 * reference to the object to draw.
 * @param s the Spline to draw
 */
SplineDrawer::SplineDrawer(Spline& s)
{
  m_spline = &s;
  m_own_spline = false;
}

/** Constructor.
 * Contrary to the other constructor, this constructor create a local
 * copy of the object to draw.
 * @param s the Spline to draw
 */
SplineDrawer::SplineDrawer(const Spline& s)
{
  m_spline = new Spline(s);
  m_own_spline = true;
}

/** Destructor. */
SplineDrawer::~SplineDrawer()
{
  if (m_own_spline)
    { delete m_spline; }
}

void
SplineDrawer::draw(Cairo::RefPtr<Cairo::Context>& context)
{
  for ( vector<Bezier>::iterator iter = m_spline->m_bezier_curves.begin();
	iter != m_spline->m_bezier_curves.end();
	++iter )
    {
      BezierDrawer d( *iter );
      d.draw(context);
    }
}

} // end namespace fawkes
