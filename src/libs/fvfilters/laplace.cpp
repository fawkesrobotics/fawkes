
/***************************************************************************
 *  laplace.cpp - Implementation of a laplace filter
 *
 *  Created: Thu Jun 16 16:30:23 2005
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

#include <fvfilters/laplace.h>

#include <core/exception.h>

#include <cmath>
#include <cstdlib>

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

/** @class FilterLaplace <fvfilters/laplace.h>
 * Laplacian filter.
 * Laplacian of Gaussian filter.
 * @author Tim Niemueller
 */

/** Constructor. */
FilterLaplace::FilterLaplace()
  : Filter("FilterLaplace")
{
  kernel = NULL;
  kernel_float = NULL;
}


/** Constructor.
 * @param sigma sigma for Laplacian
 * @param size size of kernel
 * @param scale scale factor
 */
FilterLaplace::FilterLaplace(float sigma, unsigned int size, float scale)
  : Filter("FilterLaplace")
{
  kernel_size = size;
  kernel = (int *)malloc( size * size * sizeof(int) );
  calculate_kernel( kernel, sigma, size, scale );
#ifdef HAVE_OPENCV
  kernel_float = (float *)malloc(size * size * sizeof(float));
  for (unsigned int i = 0; i < size * size; ++i) {
    kernel_float[i] = kernel[i];
  }
#endif
}


/** Destructor. */
FilterLaplace::~FilterLaplace()
{
  if ( kernel != NULL ) {
    free( kernel );
  }
  if ( kernel_float != NULL ) {
    free( kernel_float );
  }
}


void
FilterLaplace::apply()
{
#if defined(HAVE_IPP)
  IppiSize size;
  size.width = src_roi[0]->width - kernel_size;
  size.height = src_roi[0]->height - kernel_size;

  IppStatus status;

  if ( kernel == NULL ) {
    //                                   base + number of bytes to line y              + pixel bytes
    status = ippiFilterLaplace_8u_C1R( src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step), src_roi[0]->line_step,
				       dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step), dst_roi->line_step,
				       size, ippMskSize5x5 );
  } else {
    IppiSize ksize = { kernel_size, kernel_size };
    IppiPoint kanchor = { (kernel_size + 1) / 2, (kernel_size + 1) / 2 };

    /*
    std::cout << "steps:   " << src_roi[0]->line_step << "   " << dst_roi->line_step << std::endl
	      << "ksize:   " << ksize.width << " x " << ksize.height << std::endl
	      << "kanchor: " << kanchor.x << "," << kanchor.y << std::endl;
    */

    status = ippiFilter_8u_C1R( src[0] + ((src_roi[0]->start.y + kernel_size / 2) * src_roi[0]->line_step) + ((src_roi[0]->start.x + kernel_size / 2) * src_roi[0]->pixel_step), src_roi[0]->line_step,
				dst + ((dst_roi->start.y + kernel_size / 2) * dst_roi->line_step) + ((dst_roi->start.x + kernel_size / 2) * dst_roi->pixel_step), dst_roi->line_step,
				size, kernel, ksize, kanchor, 1 );

  }

  if ( status != ippStsNoErr ) {
    throw fawkes::Exception("Laplace filter failed with %i\n", status);
  }

  /*
  std::cout << "FilterLaplace: ippiFilterLaplace exit code: " << std::flush;
  switch (status) {
  case ippStsNoErr:
    std::cout << "ippStsNoErr";
    break;
  case ippStsNullPtrErr:
    std::cout << "ippStsNullPtrErr";
    break;
  case ippStsSizeErr:
    std::cout << "ippStsSizeErr";
    break;
  case ippStsStepErr:
    std::cout << "ippStsStepErr";
    break;
  case ippStsMaskSizeErr:
    std::cout << "ippStsMaskSizeErr";
    break;
  default:
    std::cout << "Unknown status " << status;
  }
  std::cout << std::endl;
  */
#elif defined(HAVE_OPENCV)
  if ((dst == NULL) || (dst == src[0])) {
    throw fawkes::Exception("OpenCV-based Sobel filter cannot be in-place");
  }

  cv::Mat srcm(src_roi[0]->height, src_roi[0]->width, CV_8UC1,
               src[0] +
                 (src_roi[0]->start.y * src_roi[0]->line_step) +
                 (src_roi[0]->start.x * src_roi[0]->pixel_step),
               src_roi[0]->line_step);

  cv::Mat dstm(dst_roi->height, dst_roi->width, CV_8UC1,
               dst +
                 (dst_roi->start.y * dst_roi->line_step) +
                 (dst_roi->start.x * dst_roi->pixel_step),
               dst_roi->line_step);

  if ( kernel_float == NULL ) {
    cv::Laplacian(srcm, dstm, /* ddepth */ CV_8UC1, /* ksize */ 5);
  } else {
    cv::Mat kernel(kernel_size, kernel_size, CV_32F, kernel_float);
    cv::Point kanchor((kernel_size + 1) / 2, (kernel_size + 1) / 2);
    cv::filter2D(srcm, dstm, /* ddepth */ -1, kernel, kanchor);
  }
#endif
}


/** Calculate a Laplacian of Gaussian kernel.
 * The kernel is calculated with the formula
 * \f[
 *   roundf( \frac{-1}{\pi * \sigma^4} * 
 *           ( 1 - \frac{w^2 + h^2}{2 * \sigma^2} )
 *           * e^{-\frac{w^2 + h^2}{2 * \sigma^2}} * \mathtt{scale} )
 * \f]			   
 *
 * @param kernel buffer contains kernel upon return
 * @param sigma sigma for formula
 * @param size kernel is of quadratic size \f$\mathtt{size} \times \mathtt{size}\f$
 * @param scale scale parameter in formula
 */
void
FilterLaplace::calculate_kernel(int *kernel, float sigma, unsigned int size, float scale)
{
  //  title "LoGFUNC__________________________________________"

  /*
  std::cout.precision( 5 );
  std::cout.width( 10 );

  std::cout << "Discrete Laplacian kernel for sigma=" << sigma
	    << " quadratic size of " << size
	    << " scaled by " << scale << std::endl;
  */
  for (int h = (-(int)(size / 2)); h <= (int)((size - 1) / 2); ++h) {
    for (int w = (-(int)(size / 2)); w <= (int)((size - 1) / 2); ++w) {
      //float v = ( (w*w + h*h - 2 * sigma * sigma) / sigma * sigma * sigma * sigma )
      //* exp( -( (w*w + h*h) / (2 * sigma * sigma) ));
      int v =  (int)roundf( - 1/( M_PI * sigma * sigma * sigma * sigma ) * 
			   ( 1 - ( (w*w + h*h) / (2 * sigma * sigma) ) )
			   * exp( -( (w*w + h*h) / (2 * sigma * sigma) )) * scale  );
      // std::cout << "   " << v << std::flush;
      kernel[ (h + (size / 2)) * size + (w + (size / 2)) ] = v;
    }
    //std::cout << std::endl;
  }

  /*
  for (int h = 0; h < size; ++h) {
    for (int w = 0; w < size; ++w) {
      std::cout << "   " << kernel[ h * size + w ] << std::flush;
    }
    std::cout << std::endl;
  }
  */

}

} // end namespace firevision
