/***************************************************************************
 *  static_list_edge_constraint.cpp - edge constraint that holds a static
 *                                    of edges to block
 *
 *  Created: Sat Jul 12 16:48:23 2014
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

#include <plugins/navgraph/constraints/timed_reservation_list_edge_constraint.h>


#include <algorithm>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphTimedReservationListEdgeConstraint <plugins/navgraph/constraints/timed_reservation_list_edge_constraint.h>
 * Constraint that holds a list of edges to block with timeouts.
 * @author Sebastian Reuter
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger logger used for debug logging
 * @param name name of edge constraint
 */
NavGraphTimedReservationListEdgeConstraint::NavGraphTimedReservationListEdgeConstraint(
  Logger *logger, std::string name)
  : NavGraphStaticListEdgeConstraint(name)
{
  logger_ = logger;
  //logger_->log_info("Timed Edge Constraint", "Created %s constraint", name.c_str() );
}


/** Constructor.
 * @param logger logger used for debug logging
 * @param name name of edge constraint
 * @param edge_list list of edges to block
 */
NavGraphTimedReservationListEdgeConstraint::NavGraphTimedReservationListEdgeConstraint(
  Logger *logger, std::string name,
  std::vector<fawkes::TopologicalMapEdge> &edge_list)
: NavGraphStaticListEdgeConstraint(name, edge_list)
{
  logger_ = logger;
}

/** Virtual empty destructor. */
NavGraphTimedReservationListEdgeConstraint::~NavGraphTimedReservationListEdgeConstraint()
{
}



} // end of namespace fawkes
