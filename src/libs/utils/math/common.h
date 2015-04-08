
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

#include <limits>
#include <cmath>

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

/** Get distance of two points.
 * This is particularly useful if not using a more powerful
 * representation like Eigen.
 * @param x1 x coordinate of first point
 * @param y1 y coordinate of first point
 * @param x2 x coordinate of second point
 * @param y2 y coordinate of second point
 * @return distance
 */
inline float
point_dist(float x1, float y1, float x2, float y2)
{
  return sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
}

/** Check if two points are different with regard to a given threshold.
 * This is particularly useful if not using a more powerful
 * representation like Eigen.
 * @param x1 x coordinate of first point
 * @param y1 y coordinate of first point
 * @param x2 x coordinate of second point
 * @param y2 y coordinate of second point
 * @param threshold the threshold to compare the distance between the
 * points to.
 * @return true if the distance of the two points is greater than or equal
 * to the given threshold, false otherwise.
 */
inline bool
points_different(float x1, float y1, float x2, float y2, float threshold = 1e-4)
{
  return sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2)) >= threshold;
}

} // end namespace fawkes

#endif
