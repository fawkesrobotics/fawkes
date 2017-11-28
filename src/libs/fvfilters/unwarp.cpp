
/***************************************************************************
 *  unwarp.cpp - Implementation of unwarp filter
 *
 *  Created: Mon Jul 25 11:22:11 2005
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

#include <fvfilters/unwarp.h>

#include <fvmodels/mirror/mirrormodel.h>
#include <fvutils/color/yuv.h>
#include <cstddef>


namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterUnwarp <fvfilters/unwarp.h>
 * Create unwarped image with given mirror model.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param mm mirror model
 */
FilterUnwarp::FilterUnwarp(MirrorModel *mm)
  : Filter("FilterUnwarp")
{
  this->mm = mm;
}


void
FilterUnwarp::apply()
{
  // destination y-plane
  unsigned char *dyp  = dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step);

  // destination u-plane
  unsigned char *dup  = YUV422_PLANAR_U_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
                                   + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2) ;
  // v-plane
  unsigned char *dvp  = YUV422_PLANAR_V_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
                                   + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2);

  // line starts
  unsigned char *ldyp  = dyp;  // destination y-plane
  unsigned char *ldup  = dup;   // u-plane
  unsigned char *ldvp  = dvp;   // v-plane

  unsigned int warp1_x = 0, warp1_y = 0,
               warp2_x = 0, warp2_y = 0;

  unsigned char py1=0, py2=0, pu1=0, pu2=0, pv1=0, pv2=0;

  for (unsigned int h = 0; h < dst_roi->height; ++h) {
    for (unsigned int w = 0; w < dst_roi->width; w += 2) {
      mm->unwarp2warp( dst_roi->start.x + w, dst_roi->start.y + h,
		       &warp1_x, &warp1_y );
      mm->unwarp2warp( dst_roi->start.x + w + 1, dst_roi->start.y + h + 1,
		       &warp2_x, &warp2_y );

      if ( (warp1_x < src_roi[0]->image_width) &&
	   (warp1_y < src_roi[0]->image_height) ) {
	// Src pixel is in original image

	YUV422_PLANAR_YUV(src[0], src_roi[0]->image_width, src_roi[0]->image_height,
			  warp1_x, warp1_y,
			  py1, pu1, pv1);

	*dyp++ = py1;
	*dup   = pu1;
	*dvp   = pv1;


	if ( (warp2_x < src_roi[0]->image_width) &&
	     (warp2_y < src_roi[0]->image_height) ) {

	  YUV422_PLANAR_YUV(src[0], src_roi[0]->image_width, src_roi[0]->image_height,
			  warp2_x, warp2_y,
			  py2, pu2, pv2);
	  
	  *dyp++ = py2;
	  *dup   = (*dup + pu2) / 2;
	  *dvp   = (*dvp + pv2) / 2;
	} else {
	  *dyp++ = 0;
	}
	dup++;
	dvp++;
      } else {

	*dyp++ = 0;
	*dup   = 0;
	*dvp   = 0;

	if ( (warp2_x < src_roi[0]->image_width) &&
	     (warp2_y < src_roi[0]->image_height) ) {

	  YUV422_PLANAR_YUV(src[0], src_roi[0]->image_width, src_roi[0]->image_height,
			  warp2_x, warp2_y,
			  py2, pu2, pv2);
	  
	  *dyp++ = py2;
	  *dup   = pu2;
	  *dvp   = pv2;
	} else {
	  *dyp++ = 0;
	}

	dup++;
	dvp++;
      }
    }

    ldyp += dst_roi->line_step;
    ldup += dst_roi->line_step;
    ldup += dst_roi->line_step;
    dyp = ldyp;
    dup = ldup;
    dvp = ldvp;
  }
}

} // end namespace firevision
