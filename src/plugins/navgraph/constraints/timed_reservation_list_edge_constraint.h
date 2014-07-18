/***************************************************************************
 *  static_list_edge_constraint.h - edge constraint that holds a static list
 *                                  of edges to block
 *
 *  Created: Fri Jul 18 21:00:48 2014
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

#ifndef __NAVGRAPH_CONSTRAINTS_TIMED_RESERVATION_LIST_EDGE_CONSTRAINT_H_
#define __NAVGRAPH_CONSTRAINTS_TIMED_RESERVATION_LIST_EDGE_CONSTRAINT_H_

#include <plugins/navgraph/constraints/static_list_edge_constraint.h>

#include <vector>
#include <string>

#include <utils/graph/topological_map_graph.h>
#include <logging/logger.h>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphTimedReservationListEdgeConstraint : public NavGraphStaticListEdgeConstraint
{
 public:
   NavGraphTimedReservationListEdgeConstraint(Logger *logger, std::string name);

   NavGraphTimedReservationListEdgeConstraint(Logger *logger, std::string name,
					      std::vector<fawkes::TopologicalMapEdge> &edge_list);

  virtual ~NavGraphTimedReservationListEdgeConstraint();

 private:
  Logger* logger_;

};

} // end namespace fawkes

#endif
