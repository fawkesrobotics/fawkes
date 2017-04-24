
/***************************************************************************
 *  polygon.h - polygon related utility methods
 *
 *  Created: Sat Jul 11 18:34:01 2015
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

#ifndef __UTILS_MATH_POLYGON_H_
#define __UTILS_MATH_POLYGON_H_

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Polygon as a vector of 2D points. */
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Polygon2D;

/** Calculate area of a polygon.
 * @param p polygon
 * @return area of polygon
 */
inline float
polygon_area(const Polygon2D &p)
{ 
  float area = 0.;

  if (p.size() < 3)  return 0.;

  size_t j = p.size() - 1;
  for (size_t i = 0;  i < p.size(); ++i) {
	  area += (p[j][0] + p[i][0]) * (p[j][1] - p[i][1]); 
	  j = i;
  }

  return fabsf(area)/2.;
}


/** Check if given point lies inside the polygon.
 * The point and polygon are assumed to be in the same X-Y plane.
 * Code based on http://www.visibone.com/inpoly/inpoly.c.txt
 * Copyright (c) 1995-1996 Galacticomm, Inc.  Freeware source code.
 * Bob Stein and Craig Yap
 * Adapted from PCL pcl::isXYPointIn2DXYPolygon()
 * @param polygon polygon to check against
 * @param point point to check if it lies within the given polygon
 * @return true if the point lies inside the polygon, false otherwise
 */
inline bool
polygon_contains(const Polygon2D &polygon, const Eigen::Vector2f &point)
{
	bool in_poly = false;
	float x1, x2, y1, y2;

	const int nr_poly_points = static_cast<int>(polygon.size());
	float xold = polygon[nr_poly_points - 1][0];
	float yold = polygon[nr_poly_points - 1][1];
	for (int i = 0; i < nr_poly_points; i++) {
		float xnew = polygon[i][0];
		float ynew = polygon[i][1];
		if (xnew > xold) {
			x1 = xold;
			x2 = xnew;
			y1 = yold;
			y2 = ynew;
		} else {
			x1 = xnew;
			x2 = xold;
			y1 = ynew;
			y2 = yold;
		}

		if ( (xnew < point[0]) == (point[0] <= xold) &&
		     (point[1] - y1) * (x2 - x1) < (y2 - y1) * (point[0] - x1) )
		{
			in_poly = !in_poly;
		}
		xold = xnew;
		yold = ynew;
	}

	// And a last check for the polygon line formed by the last and the first points
	float xnew = polygon[0][0];
	float ynew = polygon[0][1];
	if (xnew > xold) {
		x1 = xold;
		x2 = xnew;
		y1 = yold;
		y2 = ynew;
	} else {
		x1 = xnew;
		x2 = xold;
		y1 = ynew;
		y2 = yold;
	}

	if ( (xnew < point[0]) == (point[0] <= xold) &&
	     (point[1] - y1) * (x2 - x1) < (y2 - y1) * (point[0] - x1) )
	{
		in_poly = !in_poly;
	}

	return (in_poly);
}


/** Calculate centroid of polygon.
 * Note that the centroid might even lie outside for an irregular polygon.
 * @param p polygon
 * @return centroid
 */
inline Eigen::Vector2f
polygon_centroid(const Polygon2D &p)
{
	Eigen::Vector2f centroid(0.,0.);

	double area = 0.0;
	double a = 0.;

  size_t j = p.size() - 1;  // The last vertex is the 'previous' one to the first

  for (size_t i = 0;  i < p.size(); ++i) {
	  a = p[j][0]*+p[i][1] - p[i][0]*p[j][1];
	  area += a;
	  centroid[0] += (p[j][0] + p[i][0]) * a;
	  centroid[1] += (p[j][1] + p[i][1]) * a;
	  j = i;
  }

  area *= 0.5;
  centroid[0] /= (6.0 * area);
  centroid[1] /= (6.0 * area);

  return centroid;
}

} // end namespace fawkes

#endif
