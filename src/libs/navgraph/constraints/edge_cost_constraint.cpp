/***************************************************************************
 *  edge_cost_constraint.cpp - base class for edge cost constraints
 *
 *  Created: Fri Jul 18 12:11:35 2014
 *  Copyright  2014  Tim Niemueller
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

#include <navgraph/constraints/edge_cost_constraint.h>

namespace fawkes {

/** @class NavGraphEdgeCostConstraint <navgraph/constraints/edge_cost_constraint.h>
 * Constraint that can be queried for an edge cost factor.
 * @author Tim Niemueller
 *
 * @fn bool NavGraphEdgeCostConstraint::cost_factor(const fawkes::NavGraphNode &from, const fawkes::NavGraphNode &to) noexcept = 0
 * Get cost factor for given edge.
 * This method must be implemented by constraint classes. It is called
 * to determine a cost factor for an edge. That is, the path costs
 * from the originating node to the destination node are multiplied
 * with this factor and thus the chance of following that specific
 * edge is decreased. The factor must be greater or equal to 1. That
 * is a requirement to keep heuristics admissible and thus the search
 * optimal.
 *
 * Note that the nodes may be passed in either ordering, therefore
 * you should not rely on a particular order, not even for directed
 * nodes!
 *
 * Further note that the method may not throw an exception. Handle
 * this internally appropriately.
 *
 * @param from node from which the edge originates
 * @param to node to which the edge leads
 * @return cost factor, a number x >= 1.0
 *
 * @var std::string NavGraphEdgeCostConstraint::name_
 * Name of constraint.
 */

/** Constructor.
 * @param name name of edge constraint
 */
NavGraphEdgeCostConstraint::NavGraphEdgeCostConstraint(std::string &name)
{
	name_ = name;
}

/** Constructor.
 * @param name name of edge constraint
 */
NavGraphEdgeCostConstraint::NavGraphEdgeCostConstraint(const char *name)
{
	name_ = name;
}

/** Virtual empty destructor. */
NavGraphEdgeCostConstraint::~NavGraphEdgeCostConstraint()
{
}

/** Get name of constraint.
 * @return name of constraint
 */
std::string
NavGraphEdgeCostConstraint::name()
{
	return name_;
}

/** Perform compuations before graph search and to indicate re-planning.
 * The compute method is called on all constraints just before a path
 * search is performed and to check if re-planning should be tried.
 *
 * It can be used for example to cache results for the coming search
 * run. The search guarantees that for each complete search run
 * compute() is called once and only once and that no two search runs
 * overlap, i.e., compute() will not be called while another search is
 * still running.
 *
 * Constraints must indicate whether any change has occured during
 * computation or since the last compute() call through the return
 * value. This is used to determine if re-planning should be
 * attempted.
 *
 * @return true if a change has occured during computation or since
 * the last call, false otherwise
 */
bool
NavGraphEdgeCostConstraint::compute(void) noexcept
{
	return false;
}

/** Check if constraint matches name.
 * @param name name string to compare this constraints name to
 * @return true if the given name is the same as this constraint's name,
 * false otherwise
 */
bool
NavGraphEdgeCostConstraint::operator==(const std::string &name) const
{
	return name_ == name;
}

} // end of namespace fawkes
