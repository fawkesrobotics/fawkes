
/***************************************************************************
 *  geom_drawer.cpp - Drawer base class
 *
 *  Created: Thu Oct 09 15:38:19 2008
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

#include <geometry/gtk/geom_drawer.h>

/** @class fawkes::GeomDrawer <geometry/gtk/geom_drawer.h>
 * Abstract base class for all drawer classes. All objects that have
 * corresponding drawer classes can easily be drawn on a
 * GeomDrawingArea.
 * @author Daniel Beck
 */

/** @fn void fawkes::GeomDrawer::draw(Cairo::RefPtr<Cairo::Context>& context)
 * This method is called by the GeomDrawingArea. Here, derived classes
 * should implement the drawing code.
 * @param context the drawing context
 */

namespace fawkes {

/** Constructor. */
GeomDrawer::GeomDrawer()
{
}

/** Destructor. */
GeomDrawer::~GeomDrawer()
{
}

} // end namespace fawkes
