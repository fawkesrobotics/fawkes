/***************************************************************************
 *  edge_constraint.cpp - base class for edge constraints
 *
 *  Created: Sat Jul 12 14:48:02 2014
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

#include <navgraph/constraints/edge_constraint.h>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphEdgeConstraint <navgraph/constraints/edge_constraint.h>
 * Constraint that can be queried to check if an edge is blocked.
 * @author Sebastian Reuter
 * @author Tim Niemueller
 *
 * @fn bool NavGraphEdgeConstraint::blocks(const fawkes::NavGraphNode &from, const fawkes::NavGraphNode &to) throw() = 0
 * Check if constraint blocks an edge.
 * This method must be implemented by constraint classes. It is called
 * to determine if an edge should be considered blocked and therefore
 * cannot be expanded during path search.
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
 * @return true if the edge should be considered blocked, false otherwise
 *
 * @var std::string NavGraphEdgeConstraint::name_
 * Name of constraint.
 */


/** Constructor.
 * @param name name of edge constraint
 */
NavGraphEdgeConstraint::NavGraphEdgeConstraint(const std::string &name)
{
  name_   = name;
}

/** Constructor.
 * @param name name of edge constraint
 */
NavGraphEdgeConstraint::NavGraphEdgeConstraint(const char *name)
{
  name_   = name;
}


/** Virtual empty destructor. */
NavGraphEdgeConstraint::~NavGraphEdgeConstraint()
{
}

/** Get name of constraint.
 * @return name of constraint
 */
std::string
NavGraphEdgeConstraint::name()
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
NavGraphEdgeConstraint::compute(void) throw()
{
  return false;
}


/** Check if constraint matches name.
 * @param name name string to compare this constraints name to
 * @return true if the given name is the same as this constraint's name,
 * false otherwise
 */
bool
NavGraphEdgeConstraint::operator==(const std::string &name) const
{
  return name_ == name;
}


} // end of namespace fawkes
