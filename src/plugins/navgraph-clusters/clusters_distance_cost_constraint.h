/***************************************************************************
 *  clusters_static_cost_constraint.h - distance-based cost factor for
 *                                      blocked edges
 *
 *  Created: Sat Jul 19 21:17:20 2014
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

#ifndef __NAVGRAPH_CLUSTERS_CLUSTERS_DISTANCE_COST_CONSTRAINT_H_
#define __NAVGRAPH_CLUSTERS_CLUSTERS_DISTANCE_COST_CONSTRAINT_H_

#include <navgraph/constraints/edge_cost_constraint.h>

#include <list>
#include <tuple>
#include <string>
#include <Eigen/Geometry>

class NavGraphClustersThread; 

class NavGraphClustersDistanceCostConstraint : public fawkes::NavGraphEdgeCostConstraint
{
 public:
  NavGraphClustersDistanceCostConstraint(const char *name, NavGraphClustersThread *parent,
				       float cost_min, float cost_max,
				       float dist_min, float dist_max);
  virtual ~NavGraphClustersDistanceCostConstraint();

  virtual bool compute(void) throw();
  virtual float cost_factor(const fawkes::NavGraphNode &from,
			    const fawkes::NavGraphNode &to) throw();

 private:
  NavGraphClustersThread *parent_;
  float cost_min_;
  float cost_max_;
  float cost_span_;
  float dist_min_;
  float dist_max_;
  float dist_span_;
  bool valid_;
  std::list<std::tuple<std::string, std::string, Eigen::Vector2f>> blocked_;
  Eigen::Vector2f pose_;

};

#endif
