
/***************************************************************************
 *  hom_point_drawer.cpp - Drawer for the HomPoint class
 *
 *  Created: Thu Oct 09 14:34:19 2008
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

#include <geometry/gtk/hom_point_drawer.h>
#include <geometry/hom_point.h>

/** @class fawkes::HomPointDrawer <geometry/gtk/hom_point_drawer.h>
 * Drawer for HomPoint objects.
 * @author Daniel Beck
 */

/** @var fawkes::HomPointDrawer::m_point_size
 * The radius of the point.
 */

namespace fawkes {

/** Constructor.
 * @param p the HomPoint to draw
 */
HomPointDrawer::HomPointDrawer(HomPoint& p)
{
  m_hom_point = &p;
  m_point_size = 0.1;
  m_own_point = false;
}

/** Constructor.
 * @param p the HomPoint to draw
 */
HomPointDrawer::HomPointDrawer(const HomPoint& p)
{
  m_hom_point = new HomPoint(p);
  m_point_size = 0.1;
  m_own_point = true;
}

/** Destructor. */
HomPointDrawer::~HomPointDrawer()
{
  if (m_own_point)
    { delete m_hom_point; }
}

/** Set the point size with which points a drawn by this drawer.
 * @param s the point size
 */
void
HomPointDrawer::set_point_size(float s)
{
  m_point_size = s;
}

void
HomPointDrawer::draw(Cairo::RefPtr<Cairo::Context>& context)
{
  float x = m_hom_point->x();
  float y = m_hom_point->y();

  context->save();
  context->move_to(x, y);
  context->arc(x, y, m_point_size, 0.0, 2.0 * M_PI);
  context->fill();
  context->restore();
  context->stroke();
}

} // end namespace fawkes
