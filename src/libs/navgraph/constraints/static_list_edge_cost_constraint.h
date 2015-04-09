/***************************************************************************
 *  static_list_edge_cost_constraint.h - edge constraint that holds cost
 *                                       factors for edges in a static list
 *
 *  Created: Fri Jul 18 15:37:10 2014 (Ouro Branco Hotel, Joao Pessoa, Brazil)
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

#ifndef __NAVGRAPH_CONSTRAINTS_STATIC_LIST_EDGE_COST_CONSTRAINT_H_
#define __NAVGRAPH_CONSTRAINTS_STATIC_LIST_EDGE_COST_CONSTRAINT_H_

#include <navgraph/constraints/edge_cost_constraint.h>
#include <core/utils/lock_vector.h>

#include <vector>
#include <string>

#include <navgraph/navgraph.h>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphStaticListEdgeCostConstraint : public NavGraphEdgeCostConstraint
{
 public:
  NavGraphStaticListEdgeCostConstraint(std::string name);

  virtual ~NavGraphStaticListEdgeCostConstraint();

  const std::vector<std::pair<fawkes::NavGraphEdge, float>> &  edge_cost_list() const;

  void add_edge(const fawkes::NavGraphEdge &edge, const float cost_factor);
  void add_edges(const std::vector<std::pair<fawkes::NavGraphEdge, float>> &edge_costs);
  void remove_edge(const fawkes::NavGraphEdge &edge);
  void clear_edges();
  bool has_edge(const fawkes::NavGraphEdge &edge);

  virtual bool compute(void) throw();

  virtual float cost_factor(const fawkes::NavGraphNode &from,
			    const fawkes::NavGraphNode &to) throw();

 private:
  std::vector<std::pair<fawkes::NavGraphEdge, float>> edge_cost_list_;
  fawkes::LockVector<std::pair<fawkes::NavGraphEdge, float>> edge_cost_list_buffer_;
  bool modified_;

};

} // end namespace fawkes

#endif
