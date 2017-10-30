
/***************************************************************************
 *  segment_scanline.cpp - Implementation of scanline segmentation filter
 *                         This filter can be used to draw the segmentation for
 *                         all objects into a colored YUV422_PLANAR buffer
 *                         but only on the scanline model points
 *
 *  Created: Thu Jul 14 15:04:23 2005
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

#include <fvfilters/segment_scanline.h>

#include <fvmodels/color/colormodel.h>
#include <fvmodels/scanlines/scanlinemodel.h>

#include <fvutils/color/yuv.h>
#include <cstddef>


namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterScanlineSegmentation <fvfilters/segment_scanline.h>
 * Segmentation filter.
 * Visually marks pixels depending of their classification determined by the
 * supplied color model to make the segmentation visible - but only the pixels
 * at scanline points.
 * The pixels are marked with the color matching the segmentation with an
 * appropriate place holder color.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cm color model to use
 * @param slm scanline model to use
 */
FilterScanlineSegmentation::FilterScanlineSegmentation(ColorModel *cm, ScanlineModel *slm)
  : Filter("FilterScanlineSegmentation")
{
  this->cm = cm;
  this->slm = slm;
}


void
FilterScanlineSegmentation::apply()
{
  unsigned int  x = 0, y = 0;
  unsigned char   py = 0, pu = 0, pv = 0;
  unsigned char *dyp, *dup, *dvp;
  color_t c;


  slm->reset();
  while (! slm->finished()) {

    x = (*slm)->x;
    y = (*slm)->y;


    // Get source pixel values
    YUV422_PLANAR_YUV(src[0], src_roi[0]->image_width, src_roi[0]->image_height, x, y,  py,  pu,  pv);

    // destination y-plane
    dyp  = dst + (y * dst_roi->line_step) + (x * dst_roi->pixel_step);
    // destination u-plane
    dup  = YUV422_PLANAR_U_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
                                   + (((y * dst_roi->line_step) + (x * dst_roi->pixel_step)) / 2) ;
    // destination v-plane
    dvp  = YUV422_PLANAR_V_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
                                   + (((y * dst_roi->line_step) + (x * dst_roi->pixel_step)) / 2);

    c = cm->determine(py, pu, pv);

    switch (c) {
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
    ++(*slm);
  }
}

} // end namespace firevision
