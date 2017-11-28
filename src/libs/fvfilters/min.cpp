
/***************************************************************************
 *  min.cpp - implementation of min intensity filter
 *
 *  Created: Mon Jun 05 16:57:57 2006
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

#include <fvfilters/min.h>

#include <core/exceptions/software.h>
#include <fvutils/color/yuv.h>
#include <cstddef>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterMin <fvfilters/min.h>
 * Minimum filter
 * @author Tim Niemueller
 */

/** Constructor. */
FilterMin::FilterMin()
  : Filter("FilterMin", 2)
{
}


void
FilterMin::apply()
{
  if ( src[0] == NULL ) throw NullPointerException("FilterInvert: src buffer 0 is NULL");
  if ( src[1] == NULL ) throw NullPointerException("FilterInvert: src buffer 1 is NULL");
  if ( src_roi[0] == NULL ) throw NullPointerException("FilterInvert: src ROI 0 is NULL");
  if ( src_roi[1] == NULL ) throw NullPointerException("FilterInvert: src ROI 1 is NULL");

  unsigned int h = 0;
  unsigned int w = 0;

  // y-plane
  unsigned char *byp   = src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step);
  // u-plane
  unsigned char *bup   = YUV422_PLANAR_U_PLANE(src[0], src_roi[0]->image_width, src_roi[0]->image_height)
    + ((src_roi[0]->start.y * src_roi[0]->line_step) / 2 + (src_roi[0]->start.x * src_roi[0]->pixel_step) / 2) ;
  // v-plane
  unsigned char *bvp   = YUV422_PLANAR_V_PLANE(src[0], src_roi[0]->image_width, src_roi[0]->image_height)
    + ((src_roi[0]->start.y * src_roi[0]->line_step) / 2 + (src_roi[0]->start.x * src_roi[0]->pixel_step) / 2);


  // y-plane
  unsigned char *fyp   = src[1] + (src_roi[1]->start.y * src_roi[1]->line_step) + (src_roi[1]->start.x * src_roi[1]->pixel_step);
  // u-plane
  unsigned char *fup   = YUV422_PLANAR_U_PLANE(src[1], src_roi[1]->image_width, src_roi[1]->image_height)
    + ((src_roi[1]->start.y * src_roi[1]->line_step) / 2 + (src_roi[1]->start.x * src_roi[1]->pixel_step) / 2) ;
  // v-plane
  unsigned char *fvp   = YUV422_PLANAR_V_PLANE(src[1], src_roi[1]->image_width, src_roi[1]->image_height)
    + ((src_roi[1]->start.y * src_roi[1]->line_step) / 2 + (src_roi[1]->start.x * src_roi[1]->pixel_step) / 2);


  // destination y-plane
  unsigned char *dyp  = dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step);
  // destination u-plane
  unsigned char *dup   = YUV422_PLANAR_U_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
    + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2) ;
  // destination v-plane
  unsigned char *dvp   = YUV422_PLANAR_V_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
    + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2);
  
  // line starts
  unsigned char *lbyp = byp;   // y-plane
  unsigned char *lbup = fup;   // u-plane
  unsigned char *lbvp = fvp;   // v-plane
  unsigned char *lfyp = fyp;   // y-plane
  unsigned char *lfup = fup;   // u-plane
  unsigned char *lfvp = fvp;   // v-plane
  unsigned char *ldyp = dyp;  // destination y-plane
  unsigned char *ldup = dup;  // destination u-plane
  unsigned char *ldvp = dvp;  // destination v-plane

  unsigned char u1, u2, v1, v2;

  for (h = 0; (h < src_roi[1]->height) && (h < dst_roi->height); ++h) {
    for (w = 0; (w < src_roi[1]->width) && (w < dst_roi->width); w += 2) {
      if ( *byp < *fyp ) {
	*dyp++ = *byp;
	u1 = *bup;
	v1 = *bvp;
      } else {
	*dyp++ = *fyp;
	u1 = *fup;
	v1 = *fvp;
      }
      ++byp;
      ++fyp;

      if ( *byp < *fyp ) {
	*dyp++ = *byp;
	u2 = *bup;
	v2 = *bvp;
      } else {
	*dyp++ = *fyp;
	u2 = *fup;
	v2 = *fvp;
      }
      ++byp;
      ++fyp;

      *dup++ = (u1 + u2) / 2;
      *dvp++ = (v1 + v2) / 2;

      ++bup;
      ++bvp;
      ++fup;
      ++fvp;
    }

    lbyp   += src_roi[0]->line_step;
    lbup   += src_roi[0]->line_step / 2;
    lbvp   += src_roi[0]->line_step / 2;
    lfyp   += src_roi[1]->line_step;
    lfup   += src_roi[1]->line_step / 2;
    lfvp   += src_roi[1]->line_step / 2;
    ldyp  += dst_roi->line_step;
    ldup  += dst_roi->line_step / 2;
    ldvp  += dst_roi->line_step / 2;
    byp    = lbyp;
    bup    = lbup;
    bvp    = lbvp;
    fyp    = lfyp;
    fup    = lfup;
    fvp    = lfvp;
    dyp    = ldyp;
    dup    = ldup;
    dvp    = ldvp;
  }

}

} // end namespace firevision
