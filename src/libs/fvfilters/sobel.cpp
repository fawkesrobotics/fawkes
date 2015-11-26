
/***************************************************************************
 *  sobel.cpp - Implementation of a Sobel filter
 *
 *  Created: Thu May 12 13:20:43 2005
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

#include <fvfilters/sobel.h>

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

/** @class FilterSobel <fvfilters/sobel.h>
 * Sobel filter.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param ori edge orientation
 */
FilterSobel::FilterSobel(orientation_t ori)
  : Filter("FilterSobel")
{
}


/** Generate a sobel kernel for the given orientation.
 * @param k matrix for the kernel of size 3x3, contains three
 * lines concatenated into an one dimensional array.
 * @param ori requested orientation of the filter
 */
static inline void
generate_kernel(
#ifdef HAVE_IPP
                int *k,
#else
                float *k,
#endif
                orientation_t ori)
{
  // k is the kernel
  switch (ori) {
  case ORI_DEG_0:
  case ORI_DEG_360:
    k[0] =  1;    k[1] =  2;    k[2] =  1;
    k[3] =  0;    k[4] =  0;    k[5] =  0;
    k[6] = -1;    k[7] = -2;    k[8] = -1;
    break;
  case ORI_DEG_45:
    k[0] =  2;    k[1] =  1;    k[2] =  0;
    k[3] =  1;    k[4] =  0;    k[5] = -1;
    k[6] =  0;    k[7] = -1;    k[8] = -2;
    break;
  case ORI_DEG_90:
    k[0] =  1;    k[1] =  0;    k[2] = -1;
    k[3] =  2;    k[4] =  0;    k[5] = -2;
    k[6] =  1;    k[7] =  0;    k[8] = -1;
    break;
  case ORI_DEG_135:
    k[0] =  0;    k[1] = -1;    k[2] = -2;
    k[3] =  1;    k[4] =  0;    k[5] = -1;
    k[6] =  2;    k[7] =  1;    k[8] =  0;
    break;
  case ORI_DEG_180:
    k[0] = -1;    k[1] = -2;    k[2] = -1;
    k[3] =  0;    k[4] =  0;    k[5] =  0;
    k[6] =  1;    k[7] =  2;    k[8] =  1;
    break;
  case ORI_DEG_225:
    k[0] = -2;    k[1] = -1;    k[2] =  0;
    k[3] = -1;    k[4] =  0;    k[5] =  1;
    k[6] =  0;    k[7] =  1;    k[8] =  2;
    break;
  case ORI_DEG_270:
    k[0] = -1;    k[1] =  0;    k[2] =  1;
    k[3] = -2;    k[4] =  0;    k[5] =  2;
    k[6] = -1;    k[7] =  0;    k[8] =  1;
    break;
  case ORI_DEG_315:
    k[0] =  0;    k[1] =  1;    k[2] =  2;
    k[3] = -1;    k[4] =  0;    k[5] =  1;
    k[6] = -2;    k[7] = -1;    k[8] =  0;
    break;
  default:
    throw fawkes::Exception("Cannot generate Sobel kernel for the given orientation");
    break;
  }
}

void
FilterSobel::apply()
{
  shrink_region(src_roi[0], 3);
  shrink_region(dst_roi, 3);

#if defined(HAVE_IPP)
  IppiSize size;
  size.width = src_roi[0]->width;
  size.height = src_roi[0]->height;

  IppStatus status;

  if (ori[0] == ORI_HORIZONTAL) {
    //                                    base + number of bytes to line y              + pixel bytes
    status = ippiFilterSobelHoriz_8u_C1R( src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step), src_roi[0]->line_step,
					  dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step), dst_roi->line_step,
					  size );
  } else if (ori[0] == ORI_VERTICAL) {
    status = ippiFilterSobelHoriz_8u_C1R( src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step), src_roi[0]->line_step,
					  dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step), dst_roi->line_step,
					  size );

  } else if ( (ori[0] == ORI_DEG_0) ||
	      (ori[0] == ORI_DEG_45) ||
	      (ori[0] == ORI_DEG_90) ||
	      (ori[0] == ORI_DEG_135) ||
	      (ori[0] == ORI_DEG_180) ||
	      (ori[0] == ORI_DEG_225) ||
	      (ori[0] == ORI_DEG_270) ||
	      (ori[0] == ORI_DEG_315) ||
	      (ori[0] == ORI_DEG_360)
	      ) {

    Ipp32s kernel[9];
    generate_kernel(kernel, ori[0]);

    IppiSize kernel_size;
    kernel_size.width = kernel_size.height = 3;

    IppiPoint anchor;
    anchor.x = anchor.y = 1;

    status = ippiFilter_8u_C1R( src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step), src_roi[0]->line_step,
				dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step), dst_roi->line_step,
				size,
				kernel, kernel_size,
				anchor,
				/* divisor */ 1 );
    
  } else {
    // cout << "FilterSobel: Unsupported direction" << endl;
    status = ippStsNullPtrErr;
  }

  if ( status != ippStsNoErr ) {
    throw fawkes::Exception("Sobel filter failed with %i", status);
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

  if (ori[0] == ORI_HORIZONTAL) {
    if ((dst == NULL) || (dst == src[0])) {
      throw fawkes::Exception("OpenCV-based Sobel filter cannot be in-place");
    }

    cv::Sobel(srcm, dstm, /* ddepth */ -1, /* xorder */ 1, /* yorder */ 0,
              /* ksize */ 3, /* scale */ 1);
  } else if (ori[0] == ORI_VERTICAL) {
    if ((dst == NULL) || (dst == src[0])) {
      throw fawkes::Exception("OpenCV-based Sobel filter cannot be in-place");
    }

    cv::Sobel(srcm, dstm, /* ddepth */ -1, /* xorder */ 0, /* yorder */ 1,
              /* ksize */ 3, /* scale */ 1);
  } else if ( (ori[0] == ORI_DEG_0) ||
	      (ori[0] == ORI_DEG_45) ||
	      (ori[0] == ORI_DEG_90) ||
	      (ori[0] == ORI_DEG_135) ||
	      (ori[0] == ORI_DEG_180) ||
	      (ori[0] == ORI_DEG_225) ||
	      (ori[0] == ORI_DEG_270) ||
	      (ori[0] == ORI_DEG_315) ||
	      (ori[0] == ORI_DEG_360)
	      )
  {
    cv::Mat kernel(3, 3, CV_32F);
    generate_kernel((float *)kernel.ptr(), ori[0]);
 
    cv::filter2D(srcm, dstm, /* ddepth */ -1, kernel, cv::Point(1, 1));
  } else {
    throw fawkes::Exception("Unknown filter sobel orientation");

  }
#endif

}

} // end namespace firevision
