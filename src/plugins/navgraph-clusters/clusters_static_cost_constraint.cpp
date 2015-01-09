/***************************************************************************
 *  clusters_static_cost_constraint.cpp - static cost factor for blocked edges
 *
 *  Created: Fri Jul 18 22:39:30 2014
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

#include "clusters_static_cost_constraint.h"
#include "navgraph_clusters_thread.h"

/** @class NavGraphClustersStaticCostConstraint "clusters_static_cost_constraint.h"
 * Constraint apply a static cost factor to blocked edges.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param name constraint name
 * @param parent parent to call for blocked edges
 * @param cost_factor cost factor to return for blocked edges
 */
NavGraphClustersStaticCostConstraint::NavGraphClustersStaticCostConstraint(
  const char *name,
  NavGraphClustersThread *parent,
  float cost_factor)
: NavGraphEdgeCostConstraint(name)
{
  parent_      = parent;
  cost_factor_ = cost_factor;
}

/** Virtual empty destructor. */
NavGraphClustersStaticCostConstraint::~NavGraphClustersStaticCostConstraint()
{
}


bool
NavGraphClustersStaticCostConstraint::compute(void) throw()
{
  blocked_ = parent_->blocked_edges();
  return true;
}


float
NavGraphClustersStaticCostConstraint::cost_factor(const fawkes::NavGraphNode &from,
						  const fawkes::NavGraphNode &to) throw()
{
  std::string to_n = to.name();
  std::string from_n = from.name();
  if ((find(blocked_.begin(), blocked_.end(), make_pair(from_n, to_n)) != blocked_.end()) ||
      (find(blocked_.begin(), blocked_.end(), make_pair(to_n, from_n)) != blocked_.end()) )
  {
    return cost_factor_;
  }

  return 1.0;
}
