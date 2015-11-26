
/***************************************************************************
 *  or.cpp - Implementation for "or'ing" images together
 *
 *  Created: Fri May 13 14:57:10 2005
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

#include <fvfilters/or.h>

#include <core/exception.h>

#include <cstddef>

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

/** @class FilterOr <fvfilters/or.h>
 * Or filter.
 * @author Tim Niemueller
 */

/** Constructor. */
FilterOr::FilterOr()
  : Filter("FilterOr", 2)
{
}


void
FilterOr::apply()
{
#ifdef HAVE_IPP
  IppiSize size;
  size.width = src_roi[0]->width;
  size.height = src_roi[0]->height;

  IppStatus status;

  if ( (dst == NULL) || (dst == src[1]) ) {
    // In-place
    status = ippiOr_8u_C1IR(src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step),
			    src_roi[0]->line_step,
			    src[1] + (src_roi[1]->start.y * src_roi[1]->line_step) + (src_roi[1]->start.x * src_roi[1]->pixel_step),
			    src_roi[1]->line_step,
			    size);
    
  } else {
    status = ippiOr_8u_C1R(src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step),
			   src_roi[0]->line_step,
			   src[1] + (src_roi[1]->start.y * src_roi[1]->line_step) + (src_roi[1]->start.x * src_roi[1]->pixel_step),
			   src_roi[1]->line_step,
			   dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step),
			   dst_roi->line_step,
			   size);
  }

  if ( status != ippStsNoErr ) {
    throw fawkes::Exception("Or filter failed with %i\n", status);
  }
#elif defined(HAVE_OPENCV)

  if ((dst == NULL) || (dst == src[0])) {
    throw fawkes::Exception("OpenCV-based OR filter cannot be in-place");
  }

  cv::Mat srcm_0(src_roi[0]->height, src_roi[0]->width, CV_8UC1,
                 src[0] +
                   (src_roi[0]->start.y * src_roi[0]->line_step) +
                   (src_roi[0]->start.x * src_roi[0]->pixel_step),
                 src_roi[0]->line_step);

  cv::Mat srcm_1(src_roi[1]->height, src_roi[1]->width, CV_8UC1,
                 src[1] +
                   (src_roi[1]->start.y * src_roi[1]->line_step) +
                   (src_roi[1]->start.x * src_roi[1]->pixel_step),
                 src_roi[1]->line_step);

  cv::Mat dstm(dst_roi->height, dst_roi->width, CV_8UC1,
               dst +
                 (dst_roi->start.y * dst_roi->line_step) +
                 (dst_roi->start.x * dst_roi->pixel_step),
               dst_roi->line_step);

  cv::bitwise_or(srcm_0, srcm_1, dstm);

#endif
}

} // end namespace firevision
