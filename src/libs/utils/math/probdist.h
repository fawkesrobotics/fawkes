
/***************************************************************************
 *  probdist.h  probabilistic distributions
 *
 *  Created: Wed Jan 4 2009
 *  Copyright  2009 Masrur Doostdar
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

#ifndef __UTILS_MATH_PROBDIST_H_
#define __UTILS_MATH_PROBDIST_H_

#include <cmath>


namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** The normal distribution
 * @param diff the differance: (x - mu) for the mean mu and the randomvariable x
 * @param sigma the variance
 * @return probability in normal distribution
 */
inline float gauss( const float diff, 
		    const float sigma = 1.0 )
{
  return sigma==0.0 ? (diff==0.0? 1.0 : 0.0) : (1.0 / sqrtf(2.0 * M_PI)) * 1/sigma * expf( -0.5 * ( (diff*diff) / (sigma*sigma))) ;  
} 



/** Computes the intersection integral of two gaussians given
 * @param mu1 mean of first gaussian
 * @param sigma1 variance of first gaussian
 * @param mu2 mean of second gaussian
 * @param sigma2 variance of second gaussian
 * @param step discretization steps for the integral computation
 * @return computed integral
 */
inline float intersection_integral_oftwo_gaussians(float mu1,float sigma1, float mu2, float sigma2, float step){
  float begin=std::max(mu1-3*sigma1, mu2-3*sigma2);
  float end=std::min(mu1+3*sigma1, mu2+3*sigma2);
  float integral=0;
  for (float i=begin;i<end; i+=step){
    integral+=std::min(gauss(mu1-i,sigma1), gauss(mu2-i,sigma2));
  }
  integral*=step;
  return integral;
}


} // end namespace fawkes

#endif
