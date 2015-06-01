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

#ifndef __NAVGRAPH_CONSTRAINTS_POLYGON_EDGE_CONSTRAINT_H_
#define __NAVGRAPH_CONSTRAINTS_POLYGON_EDGE_CONSTRAINT_H_

#include <navgraph/constraints/edge_constraint.h>
#include <navgraph/constraints/polygon_constraint.h>

#include <vector>
#include <string>

#include <navgraph/navgraph.h>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphPolygonEdgeConstraint
: public NavGraphEdgeConstraint,
  public NavGraphPolygonConstraint
{
 public:
  NavGraphPolygonEdgeConstraint(const std::string &name);
  NavGraphPolygonEdgeConstraint(const std::string &name, const Polygon &polygon);

  virtual ~NavGraphPolygonEdgeConstraint();

  virtual bool compute(void) throw();

  virtual bool blocks(const fawkes::NavGraphNode &from,
		      const fawkes::NavGraphNode &to) throw();
};

} // end namespace fawkes

#endif
