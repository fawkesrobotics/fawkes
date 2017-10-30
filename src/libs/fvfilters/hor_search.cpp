
/***************************************************************************
 *  hor_search.cpp - Implementation of horizontal search filter
 *
 *  Created: Wed Jul 06 11:57:40 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *             2005       Yuxiao Hu (Yuxiao.Hu@rwth-aachen.de)
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

#include <fvfilters/hor_search.h>

#include <fvutils/color/yuv.h>

#include <cstddef>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterHSearch <fvfilters/hor_search.h>
 * Search horizontally for a color change. Mark these changes with white
 * pixels, all other with black pixels.
 * @author Yuxiao Hu
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cm color model to use to determine the color change
 * @param what what to look for, this color is considered as foreground,
 * all other colors are background.
 */
FilterHSearch::FilterHSearch(ColorModel *cm, color_t what)
  : Filter("FilterHSearch")
{
  this->cm = cm;
  this->what = what;
}


void
FilterHSearch::apply()
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

  // left and right boundary of the current line
  unsigned int left;
  unsigned int right;
  bool flag;

  for (h = 0; (h < src_roi[0]->height) && (h < dst_roi->height); ++h) {
    flag = false;
    left = right = 0;
    for (w = 0; (w < src_roi[0]->width) && (w < dst_roi->width); ++w) {
      if ( (cm->determine(*yp++, *up, *vp) == what) ) {
	right = w;
        flag = true;
      } else {
	left = flag?left:w;
      }
      if ( (cm->determine(*yp++, *up++, *vp++) == what) ) {
	right = ++w;
        flag = true;
      } else {
        ++w;
	left = flag?left:w;
      }
    }

    // clear the dst buffer for this line
    memset(ldyp, 0, dst_roi->width);

    // set the left- and right-most pixel to white
    // but if the pixel is at the boundary, we ignore it
    // in order to eliminate a straight line at the border.
    if (left != 0 && left < dst_roi->width) ldyp[left] = 255;
    if (right != 0 && right < dst_roi->width) ldyp[right] = 255;

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
