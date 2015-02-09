/***************************************************************************
 *  clusters_block_constraint.h - block edges close to clusters
 *
 *  Created: Fri Jul 18 21:55:25 2014
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

#include "clusters_block_constraint.h"
#include "navgraph_clusters_thread.h"

/** @class NavGraphClustersBlockConstraint "clusters_block_constraint.h"
 * Constraint to block edges close to clusters.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param name constraint name
 * @param parent parent to call for blocked edges
 */
NavGraphClustersBlockConstraint::NavGraphClustersBlockConstraint(const char *name,
								 NavGraphClustersThread *parent)
  : NavGraphEdgeConstraint(name)
{
  parent_ = parent;
}

/** Virtual empty destructor. */
NavGraphClustersBlockConstraint::~NavGraphClustersBlockConstraint()
{
}


bool
NavGraphClustersBlockConstraint::compute(void) throw()
{
  blocked_ = parent_->blocked_edges();
  return true;
}


bool
NavGraphClustersBlockConstraint::blocks(const fawkes::NavGraphNode &from,
					const fawkes::NavGraphNode &to) throw()
{
  std::string to_n = to.name();
  std::string from_n = from.name();
  return
    ((find(blocked_.begin(), blocked_.end(), make_pair(from_n, to_n)) != blocked_.end()) ||
     (find(blocked_.begin(), blocked_.end(), make_pair(to_n, from_n)) != blocked_.end()) );
}

