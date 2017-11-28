
/***************************************************************************
 *  segment.cpp - Implementation of segmentation filter
 *                This filter can be used to draw the segmentation for a
 *                given object type to the Y-plane of the image
 *
 *  Created: Mon Jun 27 11:37:57 2005
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

#include <fvmodels/color/colormodel.h>
#include <fvfilters/segment.h>

#include <fvutils/color/yuv.h>
#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterSegment <fvfilters/segment.h>
 * Segmentation filter.
 * Visually marks pixels of a given color and makes the segmentation visible.
 * The pixels are marked with bright colors.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cm color model to use
 * @param what what to mark
 */
FilterSegment::FilterSegment(ColorModel *cm, color_t what)
  : Filter("FilterSegment")
{
  this->cm = cm;
  this->what = what;
}


void
FilterSegment::apply()
{
  unsigned int h = 0;
  unsigned int w = 0;

  // y-plane
  unsigned char *yp   = src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step);
  // u-plane
  unsigned char *up   = YUV422_PLANAR_U_PLANE(src[0], src_roi[0]->image_width, src_roi[0]->image_height)
                                   + ((src_roi[0]->start.y * src_roi[0]->line_step) / 2 + (src_roi[0]->start.x * src_roi[0]->pixel_step) / 2) ;
  // v-plane
  unsigned char *vp   = YUV422_PLANAR_V_PLANE(src[0], src_roi[0]->image_width, src_roi[0]->image_height)
                                   + ((src_roi[0]->start.y * src_roi[0]->line_step) / 2 + (src_roi[0]->start.x * src_roi[0]->pixel_step) / 2);

  // destination y-plane
  unsigned char *dyp  = dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step);

  // line starts
  unsigned char *lyp  = yp;   // y-plane
  unsigned char *lup  = up;   // u-plane
  unsigned char *lvp  = vp;   // v-plane
  unsigned char *ldyp = dyp;  // destination y-plane

  for (h = 0; (h < src_roi[0]->height) && (h < dst_roi->height); ++h) {
    for (w = 0; (w < src_roi[0]->width) && (w < dst_roi->width); w += 2) {
      if ( (cm->determine(*yp++, *up, *vp) == what) ) {
	*dyp++ = 255;
      } else {
	*dyp++ =   0;
      }
      if ( (cm->determine(*yp++, *up++, *vp++) == what) ) {
	*dyp++ = 255;
      } else {
	*dyp++ =   0;
      }
    }
    lyp  += src_roi[0]->line_step;
    lup  += src_roi[0]->line_step / 2;
    lvp  += src_roi[0]->line_step / 2;
    ldyp += dst_roi->line_step;
    yp    = lyp;
    up    = lup;
    vp    = lvp;
    dyp   = ldyp;
  }

}

} // end namespace firevision
