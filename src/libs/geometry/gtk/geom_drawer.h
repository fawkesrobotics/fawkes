
/***************************************************************************
 *  geom_drawer.h - Drawer base class
 *
 *  Created: Thu Oct 09 14:23:35 2008
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

#ifndef __GEOMETRY_DRAWER_H_
#define __GEOMETRY_DRAWER_H_

#include <cairomm/cairomm.h>

namespace fawkes {
class GeomDrawer
{
 public:
  GeomDrawer();
  virtual ~GeomDrawer();

  virtual void draw(Cairo::RefPtr<Cairo::Context>& context) =0;
};

} // end namespace fawkes

#endif /* __GEOMETRY_DRAWER_H_ */
