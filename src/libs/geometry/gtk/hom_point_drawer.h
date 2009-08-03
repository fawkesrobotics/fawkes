
/***************************************************************************
 *  hom_point_drawer.h - Drawer for the HomPoint class
 *
 *  Created: Thu Oct 09 14:31:45 2008
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

#ifndef __GEOMETRY_HOM_POINT_DRAWER_H_
#define __GEOMETRY_HOM_POINT_DRAWER_H_

#include <geometry/gtk/geom_drawer.h>

namespace fawkes {

class HomPoint;

class HomPointDrawer : public GeomDrawer
{
 public:
  HomPointDrawer(HomPoint& p);
  HomPointDrawer(const HomPoint& p);
  virtual ~HomPointDrawer();

  void set_point_size(float s);

  virtual void draw(Cairo::RefPtr<Cairo::Context>& context);

 protected:
  float m_point_size;
  
 private:
  fawkes::HomPoint* m_hom_point;
  bool m_own_point;
};

} // end namespace fawkes

#endif /* __GEOMETRY_HOM_POINT_DRAWER_H_ */
