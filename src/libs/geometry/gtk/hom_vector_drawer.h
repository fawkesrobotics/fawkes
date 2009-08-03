
/***************************************************************************
 *  hom_vector_drawer.h - Drawer for the HomVector class
 *
 *  Created: Thu Oct 16 17:56:19 2008
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

#ifndef __GEOMETRY_HOM_VECTOR_DRAWER_H_
#define __GEOMETRY_HOM_VECTOR_DRAWER_H_

#include <geometry/gtk/geom_drawer.h>
#include <geometry/hom_point.h>

namespace fawkes {
class HomVector;

class HomVectorDrawer : public GeomDrawer
{
 public:
  HomVectorDrawer(HomVector& v);
  HomVectorDrawer(HomVector& v, HomPoint& offset);
  HomVectorDrawer(const HomVector& v);
  HomVectorDrawer(const HomVector& v, const HomPoint& offset);
  HomVectorDrawer(const HomVectorDrawer& d);
  virtual ~HomVectorDrawer();

  virtual void draw(Cairo::RefPtr<Cairo::Context>& context);

 private:
  HomVector* m_vector;
  HomPoint*  m_offset;
  bool       m_manager;
};


} // end namespace fawkes

#endif /* __GEOMETRY_HOM_VECTOR_DRAWER_H_ */
