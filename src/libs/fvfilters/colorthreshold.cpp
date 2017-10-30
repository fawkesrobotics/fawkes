/*
 * colorthreshold.cpp
 *
 *  Created on: 23.01.2014
 *      Author: Victor Mataré
 */

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

#include <fvfilters/colorthreshold.h>
#include <fvutils/color/rgbyuv.h>
#include <fvutils/color/yuv.h>
#include <fvutils/color/threshold.h>
#include <math.h>

namespace firevision
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


FilterColorThreshold::FilterColorThreshold(ColorModelSimilarity *color_model)
    : Filter("FilterColorThreshold", 1),
      color_model_(color_model)
{}

FilterColorThreshold::~FilterColorThreshold() {
}

void FilterColorThreshold::apply()
{
  unsigned int h = 0;
  unsigned int w = 0;

  unsigned char *p_src_y = src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step);
  unsigned char *p_src_u = YUV422_PLANAR_U_PLANE(src[0], src_roi[0]->image_width, src_roi[0]->image_height)
      + ((src_roi[0]->start.y * src_roi[0]->line_step) / 2 + (src_roi[0]->start.x * src_roi[0]->pixel_step) / 2);
  unsigned char *p_src_v = YUV422_PLANAR_V_PLANE(src[0], src_roi[0]->image_width, src_roi[0]->image_height)
      + ((src_roi[0]->start.y * src_roi[0]->line_step) / 2 + (src_roi[0]->start.x * src_roi[0]->pixel_step) / 2);

  unsigned char *p_dst_y = dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step);
  unsigned char *p_dst_u = YUV422_PLANAR_U_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
        + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2) ;
  unsigned char *p_dst_v = YUV422_PLANAR_V_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
        + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2);

  unsigned const char *p_line_src_y = p_src_y,
      *p_line_src_u = p_src_u,
      *p_line_src_v = p_src_v,
      *p_line_dst_y = p_dst_y,
      *p_line_dst_u = p_dst_u,
      *p_line_dst_v = p_dst_v;

  for (h = 0; (h < src_roi[0]->height) && (h < dst_roi->height); ++h) {
    for (w = 0; (w < src_roi[0]->width) && (w < dst_roi->width); w += 2) {

      // just copy Y plane from src to dst
      *p_dst_y++ = *p_src_y++;
      *p_dst_y++ = *p_src_y++;

      if (color_model_->determine(*p_src_y, *p_src_u, *p_src_v) != C_OTHER) {
        *p_dst_u++ = *p_src_u;
        *p_dst_v++ = *p_src_v;
      }
      else {
        *p_dst_u++ = 0x80;
        *p_dst_v++ = 0x80;
      }
      p_src_u++;
      p_src_v++;
    }

    p_line_src_y += src_roi[0]->line_step;
    p_line_src_u += src_roi[0]->line_step / 2;
    p_line_src_v += src_roi[0]->line_step / 2;

    p_line_dst_y += src_roi[0]->line_step;
    p_line_dst_u += src_roi[0]->line_step / 2;
    p_line_dst_v += src_roi[0]->line_step / 2;
  }

}


} /* namespace firevision */
