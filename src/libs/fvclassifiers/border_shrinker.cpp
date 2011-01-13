
/***************************************************************************
 *  border_shrinker.cpp - Implementation of BorderShrinker
 *
 *  Generated: Wed Feb 15 2005 15:02:56
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <fvclassifiers/border_shrinker.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/roi.h>

#include <fvmodels/scanlines/scanlinemodel.h>
#include <fvmodels/color/colormodel.h>

#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BorderShrinker <fvclassifiers/border_shrinker.h>
 * Border shrinker.
 * This shrinker makes sure that a ROI does not get too close to the image
 * boundaries. This may be needed for some mask-based operations.
 *
 */

/** Constructor.
 * @param border_left minimum x value for ROI
 * @param border_right maximum x plus width value for ROI
 * @param border_top minimum y value for ROI
 * @param border_bottom maximum y plus height value for ROI
 */
BorderShrinker::BorderShrinker(unsigned int border_left, unsigned int border_right,
			     unsigned int border_top, unsigned int border_bottom)
  : Shrinker()
{
  src = NULL;
  this->border_left   = border_left;
  this->border_right  = border_right;
  this->border_top    = border_top;
  this->border_bottom = border_bottom;
}


/** Virtual empty destructor. */
BorderShrinker::~BorderShrinker()
{
}


/** Shrink!
 * Do the actual shrinking.
 * @param roi ROI to shrink
 */
void
BorderShrinker::shrink( ROI *roi )
{
  unsigned int brdr; // border

  // bottom
  if (border_bottom > 0) {
    brdr = roi->image_height - border_bottom;
    if (roi->start.y >= brdr) {
      roi->height = 0;
    } else if ((roi->start.y + roi->height) > brdr) {
      roi->height -= (roi->start.y + roi->height) - brdr;
    }
  }

  // top
  if (border_top > 0) {
    brdr = border_top;
    if (roi->start.y <= brdr) {
      roi->height = 0;
    } else if ((roi->start.y + roi->height) < brdr) {
      roi->start.y = brdr;
      roi->height -= (roi->start.y + roi->height) - brdr;
    }
  }

  // right
  if (border_right > 0) {
    brdr = roi->image_width - border_right;
    if (roi->start.x >= brdr) {
      roi->width = 0;
    } else if ((roi->start.x + roi->width) > brdr) {
      roi->width -= (roi->start.x + roi->width) - brdr;
    }
  }

  // left
  if (border_left > 0) {
    brdr = border_left;
    if (roi->start.x <= brdr) {
      roi->width = 0;
    } else if ((roi->start.x + roi->width) < brdr) {
      roi->start.x = brdr;
      roi->width -= (roi->start.x + roi->width) - brdr;
    }
  }

}

} // end namespace firevision
