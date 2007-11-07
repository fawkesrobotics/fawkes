
/***************************************************************************
 *  laplace.cpp - Implementation of a laplace filter
 *
 *  Created: Thu Jun 16 16:30:23 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <filters/laplace.h>

#include <ippi.h>
#include <cmath>
#include <cstddef>
#include <cstdlib>

/** @class FilterLaplace <filters/laplace.h>
 * Laplacian filter.
 * Laplacian of Gaussian filter.
 * @author Tim Niemueller
 */

/** Constructor. */
FilterLaplace::FilterLaplace()
  : Filter("FilterLaplace")
{
  kernel = NULL;
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
}


/** Destructor. */
FilterLaplace::~FilterLaplace()
{
  if ( kernel != NULL ) {
    free( kernel );
  }
}


void
FilterLaplace::apply()
{
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
