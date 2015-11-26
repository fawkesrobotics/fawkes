
/***************************************************************************
 *  gauss.cpp - Implementation of a Gauss filter
 *
 *  Created: Thu May 12 09:33:55 2005
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

#include <fvfilters/gauss.h>

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

/** @class FilterGauss <fvfilters/gauss.h>
 * Gaussian filter.
 * Applies Gaussian linear filter to image (blur effect).
 */

/** Constructor. */
FilterGauss::FilterGauss()
  : Filter("FilterGauss")
{
}


void
FilterGauss::apply()
{
#if defined(HAVE_IPP)
  IppiSize size;
  size.width = src_roi[0]->width;
  size.height = src_roi[0]->height;

  /* IppStatus status = */ ippiFilterGauss_8u_C1R( src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step), src_roi[0]->line_step,
						   dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step), dst_roi->line_step,
						   size,
						   ippMskSize5x5 );

  /*
  cout << "FilterGauss: ippiFilterGauss exit code: " << flush;
  switch (status) {
  case ippStsNoErr:
    cout << "ippStsNoErr";
    break;
  case ippStsNullPtrErr:
    cout << "ippStsNullPtrErr";
    break;
  case ippStsSizeErr:
    cout << "ippStsSizeErr";
    break;
  case ippStsStepErr:
    cout << "ippStsStepErr";
    break;
  case ippStsMaskSizeErr:
    cout << "ippStsMaskSizeErr";
    break;
  default:
    cout << "Unknown status";
  }
  cout << endl;
  */

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

  cv::GaussianBlur(srcm, dstm, /* ksize */ cv::Size(5, 5), /* sigma */ 1.0);

#endif

}

} // end namespace firevision
