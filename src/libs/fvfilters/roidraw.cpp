
/***************************************************************************
 *  roidraw.cpp - Implementation of ROI draw filter
 *
 *  Created: Thu Jul 14 16:01:37 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvfilters/roidraw.h>
#include <fvutils/color/color_object_map.h>
#include <fvutils/draw/drawer.h>

#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterROIDraw <fvfilters/roidraw.h>
 * ROI Drawing filter.
 * This filter visually marks the given region of interest.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param rois optional list of ROIs to draw additionally to the dst_roi
 * @param style optional border style (default is INVERTED)
 */
FilterROIDraw::FilterROIDraw(const std::list<ROI> *rois, border_style_t style)
  : Filter("FilterROIDraw"),
  __rois(rois),
  __border_style(style)
{
  __drawer = new Drawer;
}

/** Destructor */
FilterROIDraw::~FilterROIDraw() {
  delete __drawer;
}

void
FilterROIDraw::draw_roi(const ROI *roi)
{
  if (__border_style == DASHED_HINT) {
    YUV_t hint_color = ColorObjectMap::get_color(roi->color);
    __drawer->set_buffer(dst, roi->image_width, roi->image_height);
    bool draw_black = false;
    fawkes::upoint_t end;
    end.x = std::min(roi->image_width - 1, roi->start.x + roi->width);
    end.y = std::min(roi->image_height - 1, roi->start.y + roi->height);

    //Top and bottom line
    for (unsigned int x = roi->start.x; x <= end.x ; ++x) {
      if (!(x % 2)) {
        __drawer->set_color(draw_black ? YUV_t::black() : hint_color);
        draw_black = !draw_black;
      }

      __drawer->color_point(x, roi->start.y);
      __drawer->color_point(x, end.y);
    }

    //Side lines
    for (unsigned int y = roi->start.y; y <= end.y; ++y) {
      if (!(y % 2)) {
        __drawer->set_color(draw_black ? YUV_t::black() : hint_color);
        draw_black = !draw_black;
      }

      __drawer->color_point(roi->start.x, y);
      __drawer->color_point(end.x, y);
    }
  }
  else {
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
}

void
FilterROIDraw::apply()
{
  if ( dst_roi ) {
    draw_roi(dst_roi);
  }
  if ( __rois ) {
    for (std::list<ROI>::const_iterator r = __rois->begin(); r != __rois->end(); ++r) {
      draw_roi(&(*r));
    }
  }
}


/** Set ROIs.
 * Set a list of ROIs. The list must persist as long as the filter is applied with
 * this list. Set to NULL to have it ignored again.
 * @param rois list of ROIs to draw additionally to the dst_roi.
 */
void
FilterROIDraw::set_rois(const std::list<ROI> *rois)
{
  __rois = rois;
}


/** Sets the preferred style
 * @param style The preferred style
 */
void
FilterROIDraw::set_style(border_style_t style)
{
  __border_style = style;
}

} // end namespace firevision
