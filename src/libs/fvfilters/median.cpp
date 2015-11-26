
/***************************************************************************
 *  median.cpp - Implementation of a median filter
 *
 *  Created: Mon Jun 05 15:02:36 2006
 *  Copyright  2005-2012  Tim Niemueller [www.niemueller.de]
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

#include <fvfilters/median.h>

#include <core/exception.h>

#ifdef HAVE_IPP
#  include <ippi.h>
#elif defined(HAVE_OPENCV)
#  if CV_MAJOR_VERSION < 2 || (CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION < 4)
#    include <opencv/cv.h>
#  endif
#  include <opencv/cv.hpp>
#else
#  error "Neither IPP nor OpenCV available"
#endif

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterMedian <fvfilters/median.h>
 * Median filter.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param mask_size size of median mask
 */
FilterMedian::FilterMedian(unsigned int mask_size)
  : Filter("FilterMedian")
{
  this->mask_size = mask_size;
}


void
FilterMedian::apply()
{
#if defined(HAVE_IPP)
  IppiSize size;
  size.width = src_roi[0]->width - mask_size;
  size.height = src_roi[0]->height - mask_size;

  IppiSize mask = { mask_size, mask_size };
  IppiPoint anchor = { (mask_size + 1) / 2, (mask_size + 1) / 2 };

  IppStatus status;

  //                                  base + number of bytes to line y              + pixel bytes
  status = ippiFilterMedian_8u_C1R( src[0] + ((src_roi[0]->start.y + (mask_size + 1) / 2) * src_roi[0]->line_step) + ((src_roi[0]->start.x + ( mask_size + 1) / 2) * src_roi[0]->pixel_step), src_roi[0]->line_step,
				    dst + ((dst_roi->start.y + (mask_size + 1) / 2) * dst_roi->line_step) + ((dst_roi->start.x + ( mask_size + 1) / 2) * dst_roi->pixel_step), dst_roi->line_step,
				    size, mask, anchor );

  if ( status != ippStsNoErr ) {
    throw fawkes::Exception("Median filter failed with %i\n", status);
  }
#elif defined(HAVE_OPENCV)
  cv::Mat srcm(src_roi[0]->height, src_roi[0]->width, CV_8UC1,
               src[0] +
                 (src_roi[0]->start.y * src_roi[0]->line_step) +
                 (src_roi[0]->start.x * src_roi[0]->pixel_step),
               src_roi[0]->line_step);

  if (dst == NULL) { dst = src[0]; dst_roi = src_roi[0]; }

  cv::Mat dstm(dst_roi->height, dst_roi->width, CV_8UC1,
               dst +
                 (dst_roi->start.y * dst_roi->line_step) +
                 (dst_roi->start.x * dst_roi->pixel_step),
               dst_roi->line_step);

  cv::medianBlur(srcm, dstm, mask_size);
#endif
}

} // end namespace firevision
