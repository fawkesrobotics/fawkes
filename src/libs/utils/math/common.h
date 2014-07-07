
/***************************************************************************
 *  common.h - common math helper functions
 *
 *  Created: Wed Oct 16 21:03:26 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#ifndef __UTILS_MATH_COMMON_H_
#define __UTILS_MATH_COMMON_H_

namespace fawkes {

/** Fast square multiplication.
 * @param x
 * @return x^2
 */
inline double
sqr(double x)
{
  return (x*x);
}

/** Fast square multiplication.
 * @param x
 * @return x^2
 */
inline float
sqr(float x)
{
  return (x*x);
}

/** Fast square multiplication.
 * @param x
 * @return x^2
 */
inline int
sqr(int x)
{
  return (x*x);
}

/** Fast square multiplication.
 * @param x
 * @return x^2
 */
inline unsigned long
sqr(unsigned long x)
{
  return (x*x);
}

} // end namespace fawkes

#endif
