/***************************************************************************
 *  reservation_list_node_constraint.cpp - node constraint that holds a list
 *                                    of reserved nodes
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

#include <plugins/navgraph/constraints/timed_reservation_list_node_constraint.h>

#include <algorithm>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphTimedReservationListNodeConstraint <plugins/navgraph/constraints/timed_reservation_list_node_constraint.h>
 * Constraint that holds a list of reserved nodes.
 * @author Sebastian Reuter
 */


/** Constructor.
 * @param logger Logger for debugging and infos
 * @param name name of node constraint
 */
NavGraphTimedReservationListNodeConstraint::NavGraphTimedReservationListNodeConstraint(
  Logger* logger, std::string name)
  : NavGraphStaticListNodeConstraint(name)
{
  logger_ = logger;
  logger_->log_info("Reservation Node Constraint ", "Created new constraint:%s", name.c_str() );
}



/** Constructor.
 * @param logger Logger for debugging and infos
 * @param name name of node constraint
 * @param node_list list of nodes to block
 */
NavGraphTimedReservationListNodeConstraint::NavGraphTimedReservationListNodeConstraint(
	Logger* logger,
    std::string name,
    std::vector<fawkes::TopologicalMapNode> &node_list)
  : NavGraphStaticListNodeConstraint(name, node_list)
{
  logger_ = logger;
}

/** Virtual empty destructor. */
NavGraphTimedReservationListNodeConstraint::~NavGraphTimedReservationListNodeConstraint()
{
}


} // end of namespace fawkes
