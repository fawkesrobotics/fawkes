/***************************************************************************
 *  polygon_node_constraint.cpp - Block nodes inside a polygon
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

#include <navgraph/constraints/polygon_node_constraint.h>

#include <algorithm>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphPolygonNodeConstraint <navgraph/constraints/polygon_constraint.h>
 * Constraint that blocks nodes inside a polygon.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param name name of node constraint
 */
NavGraphPolygonNodeConstraint::NavGraphPolygonNodeConstraint(const std::string &name)
  : NavGraphNodeConstraint(name)
{
}



/** Constructor.
 * @param name name of node constraint
 * @param polygon polygon to add immediately
 */
NavGraphPolygonNodeConstraint::NavGraphPolygonNodeConstraint(const std::string &name,
						     const Polygon &polygon)
  : NavGraphNodeConstraint(name), NavGraphPolygonConstraint(polygon)
{
}

/** Virtual empty destructor. */
NavGraphPolygonNodeConstraint::~NavGraphPolygonNodeConstraint()
{
}


bool
NavGraphPolygonNodeConstraint::compute(void) throw()
{
  if (! polygons_.empty()) {
    return true;
  } else {
    return false;
  }
}


bool
NavGraphPolygonNodeConstraint::blocks(const fawkes::NavGraphNode &node) throw()
{
  for (auto p : polygons_) {
    Point point(node.x(), node.y());
    if (in_poly(point, p.second)) {
      return true;
    }
  }

  return false;
}


} // end of namespace fawkes
