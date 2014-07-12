/***************************************************************************
 *  reservation_list_node_constraint.h - node constraint that holds a list
 *                                  of reserved nodes
 *
 *  Created: Sun Mar 02 10:47:35 2014
 *  Copyright  2014  Sebastian Reuter
 *             2014  Tim Niemueller
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

#ifndef __NAVGRAPH_CONSTRAINTS_RESERVATION_LIST_NODE_CONSTRAINT_H_
#define __NAVGRAPH_CONSTRAINTS_RESERVATION_LIST_NODE_CONSTRAINT_H_

#include <plugins/navgraph/constraints/static_list_node_constraint.h>

#include <vector>
#include <string>

#include <utils/graph/topological_map_graph.h>
#include <logging/logger.h>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphTimedReservationListNodeConstraint : public NavGraphStaticListNodeConstraint
{
 public:
  NavGraphTimedReservationListNodeConstraint(Logger *logger, std::string name);

  NavGraphTimedReservationListNodeConstraint(Logger *logger, std::string name,
					     std::vector<fawkes::TopologicalMapNode> &node_list);
  
  virtual ~NavGraphTimedReservationListNodeConstraint();

 private:
  Logger *logger_;

};

} // end namespace fawkes

#endif
