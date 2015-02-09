/***************************************************************************
 *  clusters_static_cost_constraint.h - static cost factor for blocked edges
 *
 *  Created: Fri Jul 18 22:36:37 2014
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

#ifndef __NAVGRAPH_CLUSTERS_CLUSTERS_STATIC_COST_CONSTRAINT_H_
#define __NAVGRAPH_CLUSTERS_CLUSTERS_STATIC_COST_CONSTRAINT_H_

#include <navgraph/constraints/edge_cost_constraint.h>

#include <list>
#include <string>

class NavGraphClustersThread; 

class NavGraphClustersStaticCostConstraint : public fawkes::NavGraphEdgeCostConstraint
{
 public:
  NavGraphClustersStaticCostConstraint(const char *name, NavGraphClustersThread *parent,
				       float cost_factor);
  virtual ~NavGraphClustersStaticCostConstraint();

  virtual bool compute(void) throw();
  virtual float cost_factor(const fawkes::NavGraphNode &from,
			    const fawkes::NavGraphNode &to) throw();

 private:
  NavGraphClustersThread *parent_;
  float cost_factor_;
  std::list<std::pair<std::string, std::string>> blocked_;

};

#endif
