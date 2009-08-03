
/***************************************************************************
 *  binomial_coefficient.h - function for computing the binomial coefficient
 *
 *  Generated: Sun Nov 04 17:29:46 2007 
 *  Copyright  2007  Martin Liebenberg
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

#ifndef __UTILS_MATH_BINOMIAL_COEFFICIENT_H_
#define __UTILS_MATH_BINOMIAL_COEFFICIENT_H_

namespace fawkes {


/** @class BinomialCoefficient <utils/math/binomial_coefficient.h>
 *  Contains method to compute the binomial coefficient.
 * 
 *  @author Martin Liebenberg
 */

class BinomialCoefficient
{
 public:
  /** Calculates the binomial coefficient.
   * @param n upper value
   * @param k lower value
   * @return the binomial coefficient of n and k
   */
  static inline unsigned int binoc(unsigned int n, unsigned int k)
  {
    unsigned int result;
    if(k == 0)
      return 1;
    if(2 * k > n)
      result = binoc(n, n - k);
    else
    {
      result = n;
      for(unsigned int i = 2; i <= k; i++)
      {
	result = result * ((n + 1 - i) / i);
      }
    }
    return result;
  }
};


} // end namespace fawkes

#endif
