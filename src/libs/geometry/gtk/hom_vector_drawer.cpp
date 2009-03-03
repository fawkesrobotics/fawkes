
/***************************************************************************
 *  hom_vector_drawer.cpp - Drawer for the HomVector class
 *
 *  Created: Thu Oct 16 18:00:59 2008
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

#include <geometry/gtk/hom_vector_drawer.h>
#include <geometry/hom_vector.h>
#include <geometry/hom_point.h>

/** @class fawkes::HomVectorDrawer <geometry/gtk/hom_vector_drawer.h>
 * Drawer for HomVector objects. In order to draw a vector an
 * additional offset point needs to be given.
 * @author Daniel Beck
 */

namespace fawkes {

/** Constructor.
 * @param v a HomVector.
 */
HomVectorDrawer::HomVectorDrawer(HomVector& v)
{
  m_vector = &v;
  m_offset = NULL;
  m_manager = false;
}

/** Constructor.
 * @param v a HomVector.
 * @param offset an offset point
 */
HomVectorDrawer::HomVectorDrawer(HomVector& v, HomPoint& offset)
{
  m_vector = &v;
  m_offset = &offset;
  m_manager = false;
}

/** Constructor.
 * This constructor creates a copy of the vector to draw.
 * @param v a HomVector
 */
HomVectorDrawer::HomVectorDrawer(const HomVector& v)
{
  m_vector = new HomVector(v);
  m_offset = NULL;
  m_manager = true;
}

/** Constructor.
 * This constructor creates copies of the vector and the offset.
 * @param v a HomVector.
 * @param offset an offset point
 */
HomVectorDrawer::HomVectorDrawer(const HomVector& v, const HomPoint& offset)
{
  m_vector = new HomVector(v);
  m_offset = new HomPoint(offset);
  m_manager = true;
}

/** Copy constructor.
 * @param d another HomVectorDrawer
 */
HomVectorDrawer::HomVectorDrawer(const HomVectorDrawer& d)
{
  m_vector = new HomVector( *d.m_vector );
  m_offset = new HomPoint( *d.m_offset );
  m_manager = true;
}

/** Destrcutor. */
HomVectorDrawer::~HomVectorDrawer()
{
  if (m_manager)
    {
      delete m_vector;
      delete m_offset;
    }
}

void
HomVectorDrawer::draw(Cairo::RefPtr<Cairo::Context>& context)
{
  context->save();

  HomPoint start, end;
  if (m_offset)
    {
      start = HomPoint( m_offset->x(), m_offset->y() );
      end   = HomPoint( m_offset->x() + m_vector->x(),
			m_offset->y() + m_vector->y() );
    }
  else
    {
      start = HomPoint( 0.0, 0.0 );
      end   = HomPoint( m_vector->x(), m_vector->y() );
    }

  context->move_to( start.x(), start.y() );
  context->line_to( end.x()  , end.y()   );
  context->arc( end.x(), end.y(), 0.06, 0.0, 2.0 * M_PI);
  context->fill();

  context->stroke();
  context->restore();
}


} // end namespace fawkes
