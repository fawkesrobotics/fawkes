/***************************************************************************
 *  polygon_constraint.cpp - 
 *
 *  Created: Mon Jan 19 11:20:31 2015 (next to Super-C waiting for demo)
 *  Copyright  2015  Tim Niemueller
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <navgraph/constraints/polygon_constraint.h>

#include <algorithm>
#include <Eigen/Geometry>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphPolygonConstraint <navgraph/constraints/polygon_constraint.h>
 * Constraint that blocks nodes within and edges touching a polygon.
 * @author Tim Niemueller
 */


/** Constructor. */
NavGraphPolygonConstraint::NavGraphPolygonConstraint()
{
  cur_polygon_handle_ = 0;
}



/** Constructor.
 * @param polygon polygon to add immediately
 */
NavGraphPolygonConstraint::NavGraphPolygonConstraint(const Polygon &polygon)
{
  cur_polygon_handle_ = 0;
  add_polygon(polygon);
}

/** Virtual empty destructor. */
NavGraphPolygonConstraint::~NavGraphPolygonConstraint()
{
}


/** Add a polygon to constraint list.
 * @param polygon Polygon to add to the list
 * @return handle for the added polygon. The handle can be used to remove
 * the polygon later.
 */
NavGraphPolygonConstraint::PolygonHandle
NavGraphPolygonConstraint::add_polygon(const NavGraphPolygonConstraint::Polygon &polygon)
{
  PolygonHandle handle = ++cur_polygon_handle_;
  polygons_[handle] = polygon;
  return handle;
}

/** Remove a polygon from the constraint list.
 * @param handle handle of polygon to remove
 */
void
NavGraphPolygonConstraint::remove_polygon(const PolygonHandle &handle)
{
  if (polygons_.find(handle) != polygons_.end()) {
    polygons_.erase(handle);
  }
}


/** Get reference to the map of polygons.
 * @return map reference of polygons
 */
const NavGraphPolygonConstraint::PolygonMap &
NavGraphPolygonConstraint::polygons() const
{
  return polygons_;
}


/** Remove all polygons. */
void
NavGraphPolygonConstraint::clear_polygons()
{
  if (! polygons_.empty()) {
    polygons_.clear();
  }
}

/** Check if given point lies inside the polygon.
 * The point and polygon are assumed to be in the same X-Y plane.
 * Code based on http://www.visibone.com/inpoly/inpoly.c.txt
 * Copyright (c) 1995-1996 Galacticomm, Inc.  Freeware source code.
 * Bob Stein and Craig Yap
 * Adapted from PCL pcl::isXYPointIn2DXYPolygon()
 * @param point point to check if it lies within the given polygon
 * @param polygon polygon to check against
 * @return true if the point lies inside the polygon, false otherwise
 */
bool
NavGraphPolygonConstraint::in_poly(const Point &point, const Polygon &polygon)
{
  bool in_poly = false;
  float x1, x2, y1, y2;

  const int nr_poly_points = static_cast<int>(polygon.size());
  float xold = polygon[nr_poly_points - 1].x;
  float yold = polygon[nr_poly_points - 1].y;
  for (int i = 0; i < nr_poly_points; i++) {
    float xnew = polygon[i].x;
    float ynew = polygon[i].y;
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

    if ( (xnew < point.x) == (point.x <= xold) &&
	 (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1) )
    {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }

  // And a last check for the polygon line formed by the last and the first points
  float xnew = polygon[0].x;
  float ynew = polygon[0].y;
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

  if ( (xnew < point.x) == (point.x <= xold) &&
       (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1) )
  {
    in_poly = !in_poly;
  }

  return (in_poly);
}


/** Check if a line segments lies on a given polygon.
 * @param p1 first point of line segment
 * @param p2 second point of line segment
 * @param polygon polygon to check against
 * @return true if the line segment lies on the polygon, false otherwise
 */
bool
NavGraphPolygonConstraint::on_poly(const Point &p1, const Point &p2, const Polygon &polygon)
{
  if (polygon.size() < 3)  return false;

  for (size_t i = 0; i < polygon.size() - 1; ++i) {
    const Point &pol1 = polygon[i  ];
    const Point &pol2 = polygon[i+1];

    const Eigen::Vector2f pp1(p1.x, p1.y);
    const Eigen::Vector2f pp2(p2.x, p2.y);
    const Eigen::Vector2f ep1(pol1.x, pol1.y);
    const Eigen::Vector2f ep2(pol2.x, pol2.y);
    const Eigen::ParametrizedLine<float,2> l1 =
      Eigen::ParametrizedLine<float,2>::Through(pp1, pp2);
    const Eigen::ParametrizedLine<float,2> l2 =
      Eigen::ParametrizedLine<float,2>::Through(ep1, ep2);
    const Eigen::Hyperplane<float, 2> lh(l2);

#if EIGEN_VERSION_AT_LEAST(3,2,0)
    const Eigen::Vector2f is = l1.intersectionPoint(lh);
#else
    const Eigen::Vector2f::Scalar ip = l1.intersection(lh);
    const Eigen::Vector2f is = l1.origin() + (l1.direction() * ip);
#endif
    const Eigen::Vector2f d1 = pp2 - pp1;
    const Eigen::Vector2f d2 = ep2 - ep1;
    const float t1 = d1.dot(is - l1.origin()) / d1.squaredNorm();
    const float t2 = d2.dot(is - l2.origin()) / d2.squaredNorm();

    if ( t1 >= 0. && t1 <= 1. && t2 >= 0. && t2 <= 1. ) {
      return true;
    }
  }

  return false;
}

} // end of namespace fawkes
