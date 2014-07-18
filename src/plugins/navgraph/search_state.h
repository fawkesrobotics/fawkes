/***************************************************************************
 *  search_state.h - Graph-based global path planning - A-Star search state
 *
 *  Created: Tue Sep 18 18:14:58 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
 *             2002  Stefan Jacobs
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

#ifndef __PLUGINS_NAVGRAPH_SEARCH_STATE_THREAD_H_
#define __PLUGINS_NAVGRAPH_SEARCH_STATE_THREAD_H_

#include <utils/search/astar_state.h>
#include <utils/graph/rcsoft_map_graph.h>
#include <plugins/navgraph/constraints/constraint_repo.h>
#include <core/utils/lockptr.h>

#include <cmath>

class NavGraphSearchState : public fawkes::AStarState
{
public:
  NavGraphSearchState(fawkes::TopologicalMapNode node, fawkes::TopologicalMapNode goal,
		      fawkes::TopologicalMapGraph *map_graph,
		      fawkes::ConstraintRepo *constraint_repo = NULL);

  NavGraphSearchState(fawkes::TopologicalMapNode node, fawkes::TopologicalMapNode goal,
		      double cost_sofar, NavGraphSearchState *parent_state,
		      fawkes::TopologicalMapGraph *map_graph,
		      fawkes::ConstraintRepo *constraint_repo = NULL);
  ~NavGraphSearchState();

  fawkes::TopologicalMapNode & node();

 private:
  size_t key() { return key_; }
  inline float  cost(const fawkes::TopologicalMapNode &d) {
    return sqrtf(powf(node_.x() - d.x(), 2) +
		 powf(node_.y() - d.y(), 2) );
  }

  float estimate();
  bool  is_goal();

  std::vector<AStarState *> children();

  // state information
  fawkes::TopologicalMapNode  node_;

  // goal information
  fawkes::TopologicalMapNode  goal_;

  fawkes::TopologicalMapGraph *map_graph_;

  fawkes::ConstraintRepo *constraint_repo_;
  bool constrained_search_;

  size_t key_;
};



#endif
