
/***************************************************************************
 *  difference.cpp - implementation of difference intensity filter
 *
 *  Created: Sun Jun 25 23:07:35 2006 (on train to Ac, father in hospital)
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

#include <fvfilters/difference.h>

#include <fvutils/color/yuv.h>
#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterDifference <fvfilters/difference.h>
 * Calculates the difference of two images.
 */

/** Constructor. */
FilterDifference::FilterDifference()
  : Filter("FilterDifference", 2)
{
}


void
FilterDifference::apply()
{
  if ( src[0] == NULL ) return;
  if ( src[1] == NULL ) return;
  if ( src_roi[0] == NULL ) return;
  if ( src_roi[1] == NULL ) return;

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

  for (h = 0; (h < src_roi[1]->height) && (h < dst_roi->height); ++h) {
    for (w = 0; (w < src_roi[1]->width) && (w < dst_roi->width); w += 2) {
      *dyp++ = ((*byp - *fyp) < 0) ? 0 : (*byp - *fyp);
      ++byp; ++fyp;
      *dyp++ = ((*byp - *fyp) < 0) ? 0 : (*byp - *fyp);
      ++byp; ++fyp;

      *dup++ = (*fup++ + *bup++) / 2;
      *dvp++ = (*fvp++ + *bvp++) / 2;
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
