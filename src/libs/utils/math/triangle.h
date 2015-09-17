
/***************************************************************************
 *  triangle.h - triangle related utility methods
 *
 *  Created: Sat Jul 11 18:04:19 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_MATH_TRIANGLE_H_
#define __UTILS_MATH_TRIANGLE_H_

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Calculate triangle area.
 * @param p0 first point of triangle
 * @param p1 second point of triangle
 * @param p2 third point of triangle
 * @return area of triangle
 */
double
triangle_area(const Eigen::Vector2f &p0, const Eigen::Vector2f &p1,
              const Eigen::Vector2f &p2)
{
	return 1.f/2.f*(-p1[1]*p2[0] + p0[1]*(-p1[0] + p2[0]) + p0[0]*(p1[1] - p2[1]) + p1[0]*p2[1]);
}


/** Check if a triangle contains a point.
 * A point is also considered to be contained if it is on the boundary
 * of the triangle.
 * @param p0 first point of triangle
 * @param p1 second point of triangle
 * @param p2 third point of triangle
 * @param p point to check with respect to the given triangle
 * @return true if the point is within or on the triangle boundaries
 */
bool
triangle_contains(const Eigen::Vector2f &p0, const Eigen::Vector2f &p1,
                  const Eigen::Vector2f &p2, const Eigen::Vector2f &p)
{
	double area_2 = 2. * triangle_area(p0, p1, p2);

	double s =
		1./area_2*(p0[1]*p2[0] - p0[0]*p2[1] + (p2[1] - p0[1])*p[0] + (p0[0] - p2[0])*p[1]);
	if (s < 0)  return false;

	double t =
		1./area_2*(p0[0]*p1[1] - p0[1]*p1[0] + (p0[1] - p1[1])*p[0] + (p1[0] - p0[0])*p[1]);
	if (t < 0)  return false;

	return s+t <= 1.;
}

} // end namespace fawkes

#endif
