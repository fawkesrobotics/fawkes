
/***************************************************************************
 *  roidraw.cpp - Implementation of ROI draw filter
 *
 *  Created: Thu Jul 14 16:01:37 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <filters/roidraw.h>

#include <cstddef>

/** @class FilterROIDraw <filters/roidraw.h>
 * ROI Drawing filter.
 * This filter visually marks the given region of interest.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param rois optional list of ROIs to draw additionally to the dst_roi
 */
FilterROIDraw::FilterROIDraw(std::list<ROI *> *rois)
  : Filter("FilterROIDraw")
{
  __rois = rois;
}


void
FilterROIDraw::draw_roi(ROI *roi)
{
  // destination y-plane
  unsigned char *dyp  = dst + (roi->start.y * roi->line_step) + (roi->start.x * roi->pixel_step);

  // line starts
  unsigned char *ldyp = dyp;  // destination y-plane

  // top border
  for (unsigned int i = roi->start.x; i < (roi->start.x + roi->width); ++i) {
    *dyp = 255 - *dyp;
    dyp++;
  }

  // left and right borders
  for (unsigned int i = roi->start.y; i < (roi->start.y + roi->height); ++i) {
    ldyp += roi->line_step;
    dyp = ldyp;
    *dyp = 255 - *dyp;
    dyp += roi->width;
    *dyp = 255 - *dyp;
  }
  ldyp += roi->line_step;
  dyp = ldyp;

  // bottom border
  for (unsigned int i = roi->start.x; i < (roi->start.x + roi->width); ++i) {
    *dyp = 255 - *dyp;
    dyp++;
  }
}

void
FilterROIDraw::apply()
{
  if ( dst_roi ) {
    draw_roi(dst_roi);
  }
  if ( __rois ) {
    for (std::list<ROI *>::iterator r = __rois->begin(); r != __rois->end(); ++r) {
      draw_roi(*r);
    }
  }
}


/** Set ROIs.
 * Set a list of ROIs. The list must persist as long as the filter is applied with
 * this list. Set to NULL to have it ignored again.
 * @param rois list of ROIs to draw additionally to the dst_roi.
 */
void
FilterROIDraw::set_rois(std::list<ROI *> *rois)
{
  __rois = rois;
}
