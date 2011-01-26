
/***************************************************************************
 *  square_shrinker.cpp - Implementation of SquareShrinker
 *
 *  Created: Tue Apr 22 2008 20:58:40 (GO2008, day 4)
 *  Copyright  2005-2008  Tim Niemueller [www.niemueller.de]
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

#include <fvclassifiers/square_shrinker.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/roi.h>

#include <fvmodels/scanlines/scanlinemodel.h>
#include <fvmodels/color/colormodel.h>

#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SquareShrinker <fvclassifiers/square_shrinker.h>
 * Square shrinker.
 * This shrinker makes sure that a ROI is always squared.
 * @author Tim Niemueller
 */

/** Constructor. */
SquareShrinker::SquareShrinker()
  : Shrinker()
{
}


/** Shrink!
 * Do the actual shrinking.
 * @param roi ROI to shrink
 */
void
SquareShrinker::shrink( ROI *roi )
{
  if ( roi->width < roi->height ) {
    roi->start.y += (roi->height - roi->width) / 2;
    roi->height = roi->width;
  } else if ( roi->width > roi->height) {
    roi->start.x += (roi->width - roi->height) / 2;
    roi->width = roi->height;
  }
}

} // end namespace firevision
