/***************************************************************************
 *  static_list_edge_cost_constraint.cpp - edge constraint that holds cost
 *                                         factors for edges in a static list
 *
 *  Created: Fri Jul 18 15:39:56 2014 (Ouro Branco Hotel, Joao Pessoa, Brazil)
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

#include <navgraph/constraints/static_list_edge_cost_constraint.h>
#include <core/exception.h>

#include <algorithm>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphStaticListEdgeCostConstraint <navgraph/constraints/static_list_edge_cost_constraint.h>
 * Constraint that hold cost factors for a static list of edges.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param name name of edge constraint
 */
NavGraphStaticListEdgeCostConstraint::NavGraphStaticListEdgeCostConstraint(std::string name)
  : NavGraphEdgeCostConstraint(name)
{
  modified_ = false;
}


/** Virtual empty destructor. */
NavGraphStaticListEdgeCostConstraint::~NavGraphStaticListEdgeCostConstraint()
{
}


bool
NavGraphStaticListEdgeCostConstraint::compute(void) throw()
{
  if (modified_) {
    modified_ = false;
    edge_cost_list_buffer_.lock();
    edge_cost_list_ = edge_cost_list_buffer_;
    edge_cost_list_buffer_.unlock();
    return true;
  } else {
    return false;
  }
}


/** Add a single edge to constraint list.
 * @param edge edge to add to constraint list
 * @param cost_factor cost factor for this edge, must be >= 1.00001
 */
void
NavGraphStaticListEdgeCostConstraint::add_edge(const fawkes::NavGraphEdge &edge,
					       float cost_factor)
{
  if (cost_factor < 1.00001) {
    throw Exception("Invalid cost factor %f, must be >= 1.00001", cost_factor);
  }
  if (! has_edge(edge)) {
    modified_ = true;
    edge_cost_list_buffer_.push_back_locked(std::make_pair(edge, cost_factor));
  }
}

/** Add multiple edges to constraint list.
 * @param edges edges to add to constraint list
 */
void
NavGraphStaticListEdgeCostConstraint::add_edges(
  const std::vector<std::pair<fawkes::NavGraphEdge, float>> &edges)
{
  for (const std::pair<NavGraphEdge, float> &ec : edges) {
    add_edge(ec.first, ec.second);
  }
}

/** Remove a single edge from the constraint list.
 * @param edge edge to remote
 */
void
NavGraphStaticListEdgeCostConstraint::remove_edge(const fawkes::NavGraphEdge &edge)
{
  std::vector<std::pair<NavGraphEdge, float>>::iterator ec
    = std::find_if(edge_cost_list_buffer_.begin(), edge_cost_list_buffer_.end(),
		   [&edge](const std::pair<fawkes::NavGraphEdge, float> &p) {
		     return p.first == edge;
		   });

  if (ec != edge_cost_list_buffer_.end()) {
    modified_ = true;
    edge_cost_list_buffer_.erase_locked(ec);
  }
}

/** Check if constraint has a specific edge.
 * @param edge edge to check
 * @return true if edge is in list, false otherwise
 */
bool
NavGraphStaticListEdgeCostConstraint::has_edge(const fawkes::NavGraphEdge &edge)
{
  return (std::find_if(edge_cost_list_buffer_.begin(), edge_cost_list_buffer_.end(),
		       [&edge](const std::pair<fawkes::NavGraphEdge, float> &p) {
			 return p.first == edge;
		       })
	  != edge_cost_list_buffer_.end());
}


/** Get list of blocked edges.
 * Note that this is the list as it is currently used on queries.
 * Any operations (adding/removing edges) that have been performed
 * without compute() being called are not reflected.
 * @return list of blocked edges
 */
const std::vector<std::pair<fawkes::NavGraphEdge, float>> &
NavGraphStaticListEdgeCostConstraint::edge_cost_list() const
{
  return edge_cost_list_;
}


/** Remove all edges. */
void
NavGraphStaticListEdgeCostConstraint::clear_edges()
{
  if (! edge_cost_list_buffer_.empty()) {
    modified_ = true;
    edge_cost_list_buffer_.clear();
  }
}


float
NavGraphStaticListEdgeCostConstraint::cost_factor(const fawkes::NavGraphNode &from,
						  const fawkes::NavGraphNode &to) throw()
{
  for (std::pair<NavGraphEdge, float> &ec : edge_cost_list_) {
    if ((ec.first.from() == from.name() && ec.first.to() == to.name()) || 
	(ec.first.from() == to.name() && ec.first.to() == from.name()) )
    {
      return ec.second;
    }
  }
  return 1.0;
}


} // end of namespace fawkes
