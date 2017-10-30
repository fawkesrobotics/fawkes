
/***************************************************************************
 *  rectify.cpp - Implementation of recification filter
 *
 *  Created: Wed Nov 07 10:51:45 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#include <fvfilters/rectify.h>

#include <core/exceptions/software.h>

#include <fvutils/rectification/rectinfo_lut_block.h>
#include <fvutils/rectification/rectinfo_block.h>
#include <fvutils/color/yuv.h>
#include <cstddef>

#include <cstdio>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterRectify <fvfilters/rectify.h>
 * Rectify image.
 * This filter can be used to use a rectification information block to rectify
 * the given image. It has special support for RectificationLutInfoBlocks by using the
 * raw data pointer for fast access. For other info blocks it will simply use the
 * RectificationInfoBlock::mapping() method to get the information.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param rib Rectification Information Block
 * @param mark_zeros if set to true mappings in the rectification info block that point
 * to (0, 0) are marked with red color (luminance value unchanged). This allows for easy
 * spotting of dead regions and may explain images that look broken. Enabled by default.
 */
FilterRectify::FilterRectify(RectificationInfoBlock *rib, bool mark_zeros)
  : Filter("FilterRectify")
{
  __rib = rib;
  __mark_zeros = mark_zeros;
}


#define FILTER_RECTIFY_ADVANCE_LINE		\
  ldyp += dst_roi->line_step;			\
  ldup += dst_roi->line_step / 2;		\
  ldvp += dst_roi->line_step / 2;		\
  dyp = ldyp;					\
  dup = ldup;					\
  dvp = ldvp;

#define FILTER_RECTIFY_ASSIGN			\
  *dyp++ = py1;					\
  *dyp++ = py2;					\
  *dup++ = (pu1 + pu2) / 2;			\
  *dvp++ = (pv1 + pv2) / 2;			\


void
FilterRectify::apply()
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

  unsigned char py1=0, py2=0, pu1=0, pu2=0, pv1=0, pv2=0;

  RectificationLutInfoBlock *rlib = dynamic_cast<RectificationLutInfoBlock *>(__rib);

  if ( rlib ) {
    if ( (rlib->pixel_width() != dst_roi->image_width) ||
	 (rlib->pixel_height() != dst_roi->image_height) ) {
      throw fawkes::IllegalArgumentException("Rectification LUT and image sizes do not match");
    }

    // we have an rectification LUT info block
    rectinfo_lut_16x16_entry_t *lut = rlib->lut_data() +
                                      dst_roi->start.y * rlib->pixel_width() +
                                      dst_roi->start.x;

    rectinfo_lut_16x16_entry_t *llut = lut;

    if ( __mark_zeros ) {
      for (unsigned int h = 0; h < dst_roi->height; ++h) {
	for (unsigned int w = 0; w < dst_roi->width; w += 2) {
	  if ( lut->x == 0 && lut->y == 0 ) {
	    py1 = YUV422_PLANAR_Y_AT(src[0], src_roi[0]->image_width, w, h);
	    pu1 = 0;
	    pv1 = 255;
	  } else {
	    YUV422_PLANAR_YUV(src[0], src_roi[0]->image_width, src_roi[0]->image_height,
			      lut->x, lut->y, py1, pu1, pv1);
	  }
	  ++lut;

	  if ( lut->x == 0 && lut->y == 0 ) {
	    py2 = YUV422_PLANAR_Y_AT(src[0], src_roi[0]->image_width, w, h);
	    pu2 = 0;
	    pv2 = 255;
	  } else {
	    YUV422_PLANAR_YUV(src[0], src_roi[0]->image_width, src_roi[0]->image_height,
			      lut->x, lut->y, py2, pu2, pv2);
	  }
	  ++lut;

	  FILTER_RECTIFY_ASSIGN;
	}

	FILTER_RECTIFY_ADVANCE_LINE;
	llut += rlib->pixel_width();
	lut = llut;
      }
    } else {
      for (unsigned int h = 0; h < dst_roi->height; ++h) {
	for (unsigned int w = 0; w < dst_roi->width; w += 2) {
	  YUV422_PLANAR_YUV(src[0], src_roi[0]->image_width, src_roi[0]->image_height,
			    lut->x, lut->y, py1, pu1, pv1);
	  ++lut;
	  YUV422_PLANAR_YUV(src[0], src_roi[0]->image_width, src_roi[0]->image_height,
			    lut->x, lut->y, py2, pu2, pv2);
	  ++lut;

	  FILTER_RECTIFY_ASSIGN;
	}

	FILTER_RECTIFY_ADVANCE_LINE;
	llut += rlib->pixel_width();
	lut = llut;
      }
    }
  } else {

    printf("Unknown info block\n");

    uint16_t ur1_x = 0, ur1_y = 0,
             ur2_x = 0, ur2_y = 0;

    if (__mark_zeros) {
      for (unsigned int h = 0; h < dst_roi->height; ++h) {
	for (unsigned int w = 0; w < dst_roi->width; w += 2) {
	  __rib->mapping(w, h, &ur1_x, &ur1_y);
	  __rib->mapping(w+1, h, &ur2_x, &ur2_y);

	  if ( (ur1_x == 0) && (ur1_y == 0) ) {
	    py1 = YUV422_PLANAR_Y_AT(src[0], src_roi[0]->image_width, w, h);
	    pu1 = 0;
	    pv1 = 255;
	  } else {
	    YUV422_PLANAR_YUV(src[0], src_roi[0]->image_width, src_roi[0]->image_height,
			      ur1_x, ur1_y, py1, pu1, pv1);
	  }
	  if ( (ur2_x == 0) && (ur2_y == 0) ) {
	    py2 = YUV422_PLANAR_Y_AT(src[0], src_roi[0]->image_width, w+1, h);
	    pu2 = 0;
	    pv2 = 255;
	  } else {
	    YUV422_PLANAR_YUV(src[0], src_roi[0]->image_width, src_roi[0]->image_height,
			      ur2_x, ur2_y, py2, pu2, pv2);
	  }

	  FILTER_RECTIFY_ASSIGN;
	}

	FILTER_RECTIFY_ADVANCE_LINE;
      }
    } else {
      for (unsigned int h = 0; h < dst_roi->height; ++h) {
	for (unsigned int w = 0; w < dst_roi->width; w += 2) {
	  __rib->mapping(w, h, &ur1_x, &ur1_y);
	  __rib->mapping(w+1, h, &ur2_x, &ur2_y);

	  YUV422_PLANAR_YUV(src[0], src_roi[0]->image_width, src_roi[0]->image_height,
			    ur1_x, ur1_y, py1, pu1, pv1);
	  YUV422_PLANAR_YUV(src[0], src_roi[0]->image_width, src_roi[0]->image_height,
			    ur2_x, ur2_y, py2, pu2, pv2);

	  FILTER_RECTIFY_ASSIGN;
	}

	FILTER_RECTIFY_ADVANCE_LINE;
      }
    }

  }
}

} // end namespace firevision
