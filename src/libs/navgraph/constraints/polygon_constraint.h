/***************************************************************************
 *  polygon_constraint.h - Block nodes and edges inside or touching a polygon
 *
 *  Created: Mon Jan 19 11:14:51 2015 (next to Super-C waiting for demo)
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

#ifndef __NAVGRAPH_CONSTRAINTS_POLYGON_CONSTRAINT_H_
#define __NAVGRAPH_CONSTRAINTS_POLYGON_CONSTRAINT_H_

#include <navgraph/constraints/static_list_node_constraint.h>
#include <navgraph/constraints/static_list_edge_constraint.h>

#include <vector>
#include <string>

#include <navgraph/navgraph.h>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphPolygonConstraint
{
 public:
  /** Simple point representation for polygon. */
  typedef struct Point_ {
    /** Constructor.
     * @param x X coordinate of point
     * @param y Y coordinate of point
     */
    Point_(float x, float y) : x(x), y(y) {}
    float x;	///< X coordinate of point
    float y;	///< Y coordinate of point
  } Point;
  /// Handle for polygon for selective removal
  typedef unsigned int                     PolygonHandle;
  /// A vector of points makes a polygon.
  typedef std::vector<Point>               Polygon;
  /// Map for accessing all polygons at once with their handles.
  typedef std::map<PolygonHandle, Polygon> PolygonMap;

  virtual ~NavGraphPolygonConstraint();

  const PolygonMap &  polygons() const;
  PolygonHandle       add_polygon(const Polygon &polygon);
  void                remove_polygon(const PolygonHandle &handle);
  void                clear_polygons();

 protected:
  NavGraphPolygonConstraint();
  NavGraphPolygonConstraint(const Polygon &polygon);

  bool in_poly(const Point &point, const Polygon &polygon);
  bool on_poly(const Point &p1, const Point &p2, const Polygon &polygon);

 protected:
  PolygonMap      polygons_;	///< currently registered polygons

 private:
  unsigned int    cur_polygon_handle_;
};

} // end namespace fawkes

#endif
