/***************************************************************************
 *  constraint_repo.h - navgraph constraint repository
 *
 *  Created: Fr Mar 14 10:47:35 2014
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

#ifndef __NAVGRAPH_CONSTRAINTS_CONSTRAINT_REPO_H_
#define __NAVGRAPH_CONSTRAINTS_CONSTRAINT_REPO_H_

#include <navgraph/constraints/node_constraint.h>
#include <navgraph/constraints/edge_constraint.h>
#include <navgraph/constraints/edge_cost_constraint.h>

#include <navgraph/navgraph_edge.h>

#include <vector>
#include <tuple>
#include <list>
#include <map>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphConstraintRepo
{
 public:
  /** List of navgraph node constraints. */
  typedef std::vector<fawkes::NavGraphNodeConstraint*> NodeConstraintList;
  /** List of navgraph edge constraints. */
  typedef std::vector<fawkes::NavGraphEdgeConstraint*> EdgeConstraintList;
  /** List of navgraph edge cost constraints. */
  typedef std::vector<fawkes::NavGraphEdgeCostConstraint*> EdgeCostConstraintList;

  NavGraphConstraintRepo();
  ~NavGraphConstraintRepo();

  void register_constraint(NavGraphNodeConstraint *constraint);
  void register_constraint(NavGraphEdgeConstraint *constraint);
  void register_constraint(NavGraphEdgeCostConstraint *constraint);
  void unregister_constraint(std::string name);

  bool has_constraint(std::string &name);
  fawkes::NavGraphNodeConstraint *      get_node_constraint(std::string &name);
  fawkes::NavGraphEdgeConstraint *      get_edge_constraint(std::string &name);
  fawkes::NavGraphEdgeCostConstraint *  get_edge_cost_constraint(std::string &name);

  const NodeConstraintList &      node_constraints() const;
  const EdgeConstraintList &      edge_constraints() const;
  const EdgeCostConstraintList &  edge_cost_constraints() const;

  bool has_constraints() const;

  bool compute();

  NavGraphNodeConstraint *     blocks(const fawkes::NavGraphNode &node);
  NavGraphEdgeConstraint *     blocks(const fawkes::NavGraphNode &from,
				      const fawkes::NavGraphNode &to);

  NavGraphEdgeCostConstraint * increases_cost(const fawkes::NavGraphNode &from,
					      const fawkes::NavGraphNode &to);

  NavGraphEdgeCostConstraint * increases_cost(const fawkes::NavGraphNode &from,
					      const fawkes::NavGraphNode &to,
					      float & cost_factor);

  float                        cost_factor(const fawkes::NavGraphNode &from,
					   const fawkes::NavGraphNode &to);

  std::map<std::string, std::string>
    blocks(const std::vector<fawkes::NavGraphNode> &nodes);

  std::map<std::pair<std::string, std::string>, std::string>
    blocks(const std::vector<fawkes::NavGraphEdge> &edges);

  std::list<std::tuple<std::string, std::string, std::string, float>>
    cost_factor(const std::vector<fawkes::NavGraphEdge> &edges);

  bool modified(bool reset_modified = false);

 private:

  NodeConstraintList node_constraints_;
  EdgeConstraintList edge_constraints_;
  EdgeCostConstraintList edge_cost_constraints_;
  bool    modified_;
};
} // namespace

#endif
