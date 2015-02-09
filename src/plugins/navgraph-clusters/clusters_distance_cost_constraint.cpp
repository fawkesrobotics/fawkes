/***************************************************************************
 *  clusters_distance_cost_constraint.cpp - distance-based cost factor for
 *                                          blocked edges
 *
 *  Created: Sat Jul 19 21:17:25 2014
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

#include "clusters_distance_cost_constraint.h"
#include "navgraph_clusters_thread.h"

#include <core/exception.h>

using namespace fawkes;

/** @class NavGraphClustersDistanceCostConstraint "clusters_distance_cost_constraint.h"
 * Constraint apply linearly scaled costs based on the distance.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param name constraint name
 * @param parent parent to call for blocked edges
 * @param cost_min minimum cost for centroids in range
 * @param cost_max maximum cost for close centroids
 * @param dist_min minimum distance, for cost scaling, anything closer will be
 * assigned cost_max
 * @param dist_max maximum distance for costs, anything farther will be
 * assigned 1.0.
 */
NavGraphClustersDistanceCostConstraint::NavGraphClustersDistanceCostConstraint(
  const char *name,
  NavGraphClustersThread *parent,
  float cost_min, float cost_max, float dist_min, float dist_max)
: NavGraphEdgeCostConstraint(name)
{
  parent_    = parent;
  cost_min_  = cost_min;
  cost_max_  = cost_max;
  cost_span_ = cost_max_ - cost_min_;
  dist_min_  = dist_min;
  dist_max_  = dist_max;
  dist_span_ = dist_max_ - dist_min_;
  valid_     = false;

  if (cost_min_ > cost_max_) {
    throw Exception("Cost min must be less or equal to max");
  }
  if (dist_min_ > dist_max_) {
    throw Exception("Dist min must be less or equal to max");
  }
}

/** Virtual empty destructor. */
NavGraphClustersDistanceCostConstraint::~NavGraphClustersDistanceCostConstraint()
{
}


bool
NavGraphClustersDistanceCostConstraint::compute(void) throw()
{
  blocked_ = parent_->blocked_edges_centroids();
  valid_   = parent_->robot_pose(pose_);
  return valid_;
}


float
NavGraphClustersDistanceCostConstraint::cost_factor(const fawkes::NavGraphNode &from,
						  const fawkes::NavGraphNode &to) throw()
{
  if (valid_) {
    std::string to_n = to.name();
    std::string from_n = from.name();

    std::list<std::tuple<std::string, std::string, Eigen::Vector2f>>::iterator bl=
      std::find_if(blocked_.begin(), blocked_.end(),
		   [&to_n, &from_n](std::tuple<std::string, std::string, Eigen::Vector2f> &b) {
		     return
		     (to_n == std::get<0>(b) && from_n == std::get<1>(b)) ||
		     (to_n == std::get<1>(b) && from_n == std::get<0>(b));
		   });

    if (bl != blocked_.end()) {
      float distance = (pose_ - std::get<2>(*bl)).norm();
      if (distance <= dist_min_) {
	return cost_max_;
      } else if (distance >= dist_max_) {
	return 1.0;
      } else {
	return cost_max_ - (((distance - dist_min_) / dist_span_) * cost_span_);
      }
    }
  }

  return 1.0;
}
