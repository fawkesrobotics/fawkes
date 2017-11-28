
/***************************************************************************
 *  segment_color.cpp - Implementation of color segmentation filter
 *                      This filter can be used to draw the segmentation for
 *                      all objects into a colored YUV422_PLANAR buffer
 *
 *  Created: Mon Jul 04 16:18:15 2005
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

#include <fvfilters/segment_color.h>

#include <fvmodels/color/colormodel.h>
#include <fvutils/color/yuv.h>
#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterColorSegmentation <fvfilters/segment_color.h>
 * Segmentation filter.
 * Visually marks pixels depending of their classification determined by the
 * supplied color model to make the segmentation visible.
 * The pixels are marked with the color matching the segmentation with an
 * appropriate place holder color
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cm color model to use
 */
FilterColorSegmentation::FilterColorSegmentation(ColorModel *cm)
  : Filter("FilterColorSegmentation")
{
  this->cm = cm;
}


void
FilterColorSegmentation::apply()
{
  unsigned int h = 0;
  unsigned int w = 0;

  // source y-plane
  unsigned char *yp   = src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step);
  // source u-plane
  unsigned char *up   = YUV422_PLANAR_U_PLANE(src[0], src_roi[0]->image_width, src_roi[0]->image_height)
                                   + ((src_roi[0]->start.y * src_roi[0]->line_step) / 2 + (src_roi[0]->start.x * src_roi[0]->pixel_step) / 2) ;
  // source v-plane
  unsigned char *vp   = YUV422_PLANAR_V_PLANE(src[0], src_roi[0]->image_width, src_roi[0]->image_height)
                                   + ((src_roi[0]->start.y * src_roi[0]->line_step) / 2 + (src_roi[0]->start.x * src_roi[0]->pixel_step) / 2);

  // destination y-plane
  unsigned char *dyp  = dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step);
  // destination u-plane
  unsigned char *dup  = YUV422_PLANAR_U_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
                                   + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2) ;
  // destination v-plane
  unsigned char *dvp  = YUV422_PLANAR_V_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
                                   + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2);

  // line starts
  unsigned char *lyp  = yp;   // source y-plane
  unsigned char *lup  = up;   // source u-plane
  unsigned char *lvp  = vp;   // source v-plane
  unsigned char *ldyp = dyp;  // destination y-plane
  unsigned char *ldup = dup;  // destination y-plane
  unsigned char *ldvp = dvp;  // destination y-plane

  color_t c1;
  // Unused for now: color_t c2;

  for (h = 0; (h < src_roi[0]->height) && (h < dst_roi->height); ++h) {
    for (w = 0; (w < src_roi[0]->width) && (w < dst_roi->width); w += 2) {
      c1 = cm->determine(*yp++, *up++, *vp++);
      yp++;
       //c2 = cm->determine(*yp++, *up++, *vp++);

      switch (c1) {
      case C_ORANGE:
	*dyp++ = 128;
	*dyp++ = 128;
	*dup++ = 0;
	*dvp++ = 255;
	break;
      case C_MAGENTA:
	*dyp++ = 128;
 	*dyp++ = 128;
	*dup++ = 128;
	*dvp++ = 255;
	break;
      case C_CYAN:
	*dyp++ = 128;
 	*dyp++ = 128;
	*dup++ = 255;
	*dvp++ = 0;
	break;
      case C_BLUE:
	*dyp++ = 128;
 	*dyp++ = 128;
	*dup++ = 255;
	*dvp++ = 128;
	break;
      case C_YELLOW:
	*dyp++ = 255;
 	*dyp++ = 255;
	*dup++ = 0;
	*dvp++ = 128;
	break;
      case C_GREEN:
	*dyp++ = 128;
 	*dyp++ = 128;
	*dup++ = 0;
	*dvp++ = 0;
	break;
      case C_WHITE:
	*dyp++ = 255;
 	*dyp++ = 255;
	*dup++ = 128;
	*dvp++ = 128;
	break;
      case C_RED:
	*dyp++ = 196;
 	*dyp++ = 196;
	*dup++ = 0;
	*dvp++ = 255;
	break;
      default:
	*dyp++ = 0;
 	*dyp++ = 0;
	*dup++ = 128;
	*dvp++ = 128;
	break;
      }
    }
    lyp  += src_roi[0]->line_step;
    lup  += src_roi[0]->line_step / 2;
    lvp  += src_roi[0]->line_step / 2;
    ldyp += dst_roi->line_step;
    ldup += dst_roi->line_step / 2;
    ldvp += dst_roi->line_step / 2;
    yp    = lyp;
    up    = lup;
    vp    = lvp;
    dyp   = ldyp;
    dup   = ldup;
    dvp   = ldvp;
  }

}

} // end namespace firevision
