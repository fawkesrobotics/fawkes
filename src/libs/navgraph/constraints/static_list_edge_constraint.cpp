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

#include <navgraph/constraints/static_list_edge_constraint.h>

#include <algorithm>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphStaticListEdgeConstraint <navgraph/constraints/static_list_edge_constraint.h>
 * Constraint that holds a list of edges to block.
 * @author Sebastian Reuter
 * @author Tim Niemueller
 */


/** Constructor.
 * @param name name of edge constraint
 */
NavGraphStaticListEdgeConstraint::NavGraphStaticListEdgeConstraint(std::string name)
  : NavGraphEdgeConstraint(name)
{
  modified_ = false;
}



/** Constructor.
 * @param name name of edge constraint
 * @param edge_list list of edges to block
 */
NavGraphStaticListEdgeConstraint::NavGraphStaticListEdgeConstraint(
    std::string name,
    std::vector<fawkes::NavGraphEdge> &edge_list)
  : NavGraphEdgeConstraint(name)
{
  edge_list_ = edge_list;
  modified_ = false;
}

/** Virtual empty destructor. */
NavGraphStaticListEdgeConstraint::~NavGraphStaticListEdgeConstraint()
{
}


bool
NavGraphStaticListEdgeConstraint::compute(void) throw()
{
  if (modified_) {
    modified_ = false;
    return true;
  } else {
    return false;
  }
}


/** Add a single edge to constraint list.
 * @param edge edge to add to constraint list
 */
void
NavGraphStaticListEdgeConstraint::add_edge(const fawkes::NavGraphEdge &edge)
{
  if (! has_edge(edge)) {
    modified_ = true;
    edge_list_.push_back(edge);
  }
}

/** Add multiple edges to constraint list.
 * @param edges edges to add to constraint list
 */
void
NavGraphStaticListEdgeConstraint::add_edges(
  const std::vector<fawkes::NavGraphEdge> &edges)
{
  for (const NavGraphEdge &n : edges) {
    add_edge(n);
  }
}

/** Remove a single edge from the constraint list.
 * @param edge edge to remote
 */
void
NavGraphStaticListEdgeConstraint::remove_edge(const fawkes::NavGraphEdge &edge)
{
  std::vector<NavGraphEdge>::iterator e
    = std::find(edge_list_.begin(), edge_list_.end(), edge);
  if (e != edge_list_.end()) {
    modified_ = true;
    edge_list_.erase(e);
  }
}

/** Check if constraint has a specific edge.
 * @param edge edge to check
 * @return true if edge is in list, false otherwise
 */
bool
NavGraphStaticListEdgeConstraint::has_edge(const fawkes::NavGraphEdge &edge)
{
  return (std::find(edge_list_.begin(), edge_list_.end(), edge) != edge_list_.end());
}


/** Get list of blocked edges.
 * @return list of blocked edges
 */
const std::vector<fawkes::NavGraphEdge> &
NavGraphStaticListEdgeConstraint::edge_list() const
{
  return edge_list_;
}


/** Remove all edges. */
void
NavGraphStaticListEdgeConstraint::clear_edges()
{
  if (! edge_list_.empty()) {
    modified_ = true;
    edge_list_.clear();
  }
}


bool
NavGraphStaticListEdgeConstraint::blocks(const fawkes::NavGraphNode &from,
					 const fawkes::NavGraphNode &to) throw()
{
  for (NavGraphEdge &e : edge_list_) {
    if ((e.from() == from.name() && e.to() == to.name()) || 
	(e.from() == to.name() && e.to() == from.name()) )
    {
      return true;
    }
  }
  return false;
}


} // end of namespace fawkes
