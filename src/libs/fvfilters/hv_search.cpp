
/***************************************************************************
 *  hv_search.cpp - Implementation of horizontal- and vertical-search filter
 *
 *  Created: Tue Jul 12 14:40:40 2005
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

#include <fvfilters/hv_search.h>

#include <fvutils/color/yuv.h>

#include <cstddef>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterHVSearch <fvfilters/hv_search.h>
 * Horizontal/vertical search filter.
 * This filter works similar to the horizontal search filter, but additionally
 * it search for color changes in vertical direction.
 * @author Yuxiao Hu
 * @author Tim Niemueller
 */


/** Constructor.
 * @param cm color model to use to determine the color change
 * @param what what to look for, this color is considered as foreground,
 * all other colors are background.
 */
FilterHVSearch::FilterHVSearch(ColorModel *cm, color_t what)
  : Filter("FilterHVSearch")
{
  this->cm = cm;
  this->what = what;
}


void
FilterHVSearch::apply()
{
  unsigned int h = 0;
  unsigned int w = 0;

  unsigned int width = src_roi[0]->width <= dst_roi->width ? src_roi[0]->width : dst_roi->width;

  // Here use array to avoid overhead of dynamic mem allocation.
  unsigned int top[width];
  unsigned int bottom[width];
  bool vflag[width];

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

  // Confidence threshold for a line as "what" inside.
  const unsigned int	MIN_INTERIA	= 9;
  unsigned int		num_what;

  // Remember the widest orange line,
  // and if the following lines shrink dramatically,
  // we stop searching. This help eliminate reflextion.
  const unsigned int	MAX_SHRINK	= 16;
  unsigned int		max_width	= 0;
  bool			not_reflect	= true;

  memset(top, 0, width * sizeof(unsigned int));
  memset(bottom, 0, width * sizeof(unsigned int));
  memset(vflag, 0, width * sizeof(bool));

  for (h = 0; (h < src_roi[0]->height) && (h < dst_roi->height); ++h) {
    flag = false;
    left = right = 0;
    num_what = 0;
    for (w = 0; (w < src_roi[0]->width) && (w < dst_roi->width); ++w) {
      if ( (cm->determine(*yp++, *up, *vp) == what) ) {
	right = w;
	if (not_reflect) bottom[w] = h;
        flag = true;
	vflag[w] = true;
        ++num_what;
      } else {
	left = flag?left:w;
	if (!vflag[w]) top[w] = h;
      }
      if ( (cm->determine(*yp++, *up++, *vp++) == what) ) {
	right = ++w;
	if (not_reflect) bottom[w] = h;
        flag = true;
	vflag[w] = true;
        ++num_what;
      } else {
        ++w;
	left = flag?left:w;
	if (!vflag[w]) top[w] = h;
      }
    }

    // clear the dst buffer for this line
    memset(ldyp, 0, dst_roi->width);

    if (num_what * MIN_INTERIA > right - left)
    {
      if (right - left > max_width)
        max_width = right - left;
      if (not_reflect)
      {
        if (right - left < max_width / MAX_SHRINK)
        {
          // cout << "In line:" << h << " \tleft = " << left
          //      << " \tright = " << right << " \tmax_width = "
          //      << max_width << endl;
          not_reflect = false; // the reflection begins from here
        }

        // set the left- and right-most pixel to white
        // but if the pixel is at the boundary, we ignore it
        // in order to eliminate a straight line at the border.
        if (left != 0 && left < dst_roi->width-1)
        {
          ldyp[left] = 255;
          // ldyp[left+1] = 255;
        }
        if (right != 0 && right < dst_roi->width-1)
        {
          ldyp[right] = 255;
          // ldyp[right-1] = 255;
        }
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
  for (w = 0; w < dst_roi->width; w++)
  {
    if (top[w] != 0 && top[w] != dst_roi->height - 1)
	*(dst + ((dst_roi->start.y + top[w]) * dst_roi->line_step)
              + ((dst_roi->start.x + w) * dst_roi->pixel_step)) = 255;
    if (bottom[w] != 0 && bottom[w] != dst_roi->height - 1)
	*(dst + ((dst_roi->start.y + bottom[w]) * dst_roi->line_step)
              + ((dst_roi->start.x + w) * dst_roi->pixel_step)) = 255;
  }
}

} // end namespace firevision
