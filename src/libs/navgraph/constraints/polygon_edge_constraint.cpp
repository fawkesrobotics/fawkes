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

#include <navgraph/constraints/polygon_edge_constraint.h>

#include <algorithm>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphPolygonEdgeConstraint <navgraph/constraints/polygon_constraint.h>
 * Constraint that blocks nodes within and edges touching a polygon.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param name name of node constraint
 */
NavGraphPolygonEdgeConstraint::NavGraphPolygonEdgeConstraint(const std::string &name)
  : NavGraphEdgeConstraint(name)
{
}



/** Constructor.
 * @param name name of node constraint
 * @param polygon polygon to add immediately
 */
NavGraphPolygonEdgeConstraint::NavGraphPolygonEdgeConstraint(const std::string &name,
							     const Polygon &polygon)
  : NavGraphEdgeConstraint(name), NavGraphPolygonConstraint(polygon)
{
}

/** Virtual empty destructor. */
NavGraphPolygonEdgeConstraint::~NavGraphPolygonEdgeConstraint()
{
}


bool
NavGraphPolygonEdgeConstraint::compute(void) throw()
{
  if (! polygons_.empty()) {
    return true;
  } else {
    return false;
  }
}

bool
NavGraphPolygonEdgeConstraint::blocks(const fawkes::NavGraphNode &from,
				      const fawkes::NavGraphNode &to) throw()
{
  for (auto p : polygons_) {
    Point from_p(from.x(), from.y());
    Point to_p(to.x(), to.y());
    if (on_poly(from_p, to_p, p.second)) {
      return true;
    }
  }

  return false;
}



} // end of namespace fawkes
