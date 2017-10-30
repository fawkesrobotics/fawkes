
/***************************************************************************
 *  invert.cpp - implementation of invert filter
 *
 *  Created: Mon Jun 05 12:47:18 2006
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

#include <fvfilters/invert.h>

#include <core/exceptions/software.h>
#include <fvutils/color/yuv.h>
#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterInvert <fvfilters/invert.h>
 * Inversion filter.
 * This will invert the given image.
 * @author Tim Niemueller
 */

/** Constructor. */
FilterInvert::FilterInvert()
  : Filter("FilterInvert")
{
}


void
FilterInvert::apply()
{
  if ( src[0] == NULL )     throw fawkes::NullPointerException("FilterInvert: src buffer is NULL");
  if ( src_roi[0] == NULL ) throw fawkes::NullPointerException("FilterInvert: src ROI is NULL");

  if ( (dst == NULL) || (dst == src[0]) ) {
    // In-place

    unsigned int h = 0;
    unsigned int w = 0;

    // y-plane
    unsigned char *yp = src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step);
    
    // line starts
    unsigned char *lyp  = yp;   // y-plane

    for (h = 0; h < src_roi[0]->height; ++h) {
      for (w = 0; w < src_roi[0]->width; ++w) {
	*yp = 255 - *yp;
	++yp;
      }
      lyp  += src_roi[0]->line_step;
      yp    = lyp;
    }

  } else {

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
    // destination u-plane
    unsigned char *dup   = YUV422_PLANAR_U_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
      + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2) ;
    // destination v-plane
    unsigned char *dvp   = YUV422_PLANAR_V_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
      + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2);

    // line starts
    unsigned char *lyp  = yp;   // y-plane
    unsigned char *lup  = up;   // u-plane
    unsigned char *lvp  = vp;   // v-plane
    unsigned char *ldyp = dyp;  // destination y-plane
    unsigned char *ldup = dup;  // destination u-plane
    unsigned char *ldvp = dvp;  // destination v-plane

    for (h = 0; (h < src_roi[0]->height) && (h < dst_roi->height); ++h) {
      for (w = 0; (w < src_roi[0]->width) && (w < dst_roi->width); w += 2) {
	*dyp++ = 255 - *yp++;
	*dyp++ = 255 - *yp++;
	*dup++ = *up++;
	*dvp++ = *vp++;
      }

      lyp   += src_roi[0]->line_step;
      lup   += src_roi[0]->line_step / 2;
      lvp   += src_roi[0]->line_step / 2;
      ldyp  += dst_roi->line_step;
      ldup  += dst_roi->line_step / 2;
      ldvp  += dst_roi->line_step / 2;
      yp     = lyp;
      up     = lup;
      vp     = lvp;
      dyp    = ldyp;
      dup    = ldup;
      dvp    = ldvp;
    }
  }

}

} // end namespace firevision
