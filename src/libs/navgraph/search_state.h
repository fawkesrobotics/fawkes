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

#ifndef __LIBS_NAVGRAPH_SEARCH_STATE_H_
#define __LIBS_NAVGRAPH_SEARCH_STATE_H_

#include <utils/search/astar_state.h>
#include <navgraph/constraints/constraint_repo.h>
#include <navgraph/navgraph.h>
#include <core/utils/lockptr.h>

#include <functional>
#include <cmath>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphSearchState : public fawkes::AStarState
{
 public:
  NavGraphSearchState(fawkes::NavGraphNode node, fawkes::NavGraphNode goal,
		      fawkes::NavGraph *map_graph,
		      fawkes::NavGraphConstraintRepo *constraint_repo = NULL);

  NavGraphSearchState(fawkes::NavGraphNode node, fawkes::NavGraphNode goal,
		      fawkes::NavGraph *map_graph,
		      navgraph::EstimateFunction estimate_func,
		      navgraph::CostFunction cost_func = NavGraphSearchState::euclidean_cost,
		      fawkes::NavGraphConstraintRepo *constraint_repo = NULL);

  ~NavGraphSearchState();

  fawkes::NavGraphNode & node();

  virtual size_t key() { return key_; }
  virtual float  estimate();
  virtual bool   is_goal();

  /** Determine euclidean cost between two nodes.
   * Note that the given notes are assumed to be adjacent nodes.
   * @param from originating node
   * @param to destination node
   * @return cost from @p from to @p to.
   */
  static float
  euclidean_cost(const fawkes::NavGraphNode &from,
		 const fawkes::NavGraphNode &to)
  {
    return sqrtf(powf(to.x() - from.x(), 2) +
		 powf(to.y() - from.y(), 2) );
  }

  /** Determine straight line estimate between two nodes.
   * @param node node to query heuristic value for
   * @param goal goal node to get estimate for
   * @return estimate of cost from @p node to @p goal.
   */
  static float
  straight_line_estimate(const fawkes::NavGraphNode &node,
			 const fawkes::NavGraphNode &goal)
  {
    return sqrtf(powf(goal.x() - node.x(), 2) +
		 powf(goal.y() - node.y(), 2) );
  }

 private:
  NavGraphSearchState(fawkes::NavGraphNode node, fawkes::NavGraphNode goal,
		      double cost_sofar, NavGraphSearchState *parent_state,
		      fawkes::NavGraph *map_graph,
		      navgraph::EstimateFunction estimate_func,
		      navgraph::CostFunction cost_func,
		      fawkes::NavGraphConstraintRepo *constraint_repo = NULL);

 private:
  std::vector<AStarState *> children();

  // state information
  fawkes::NavGraphNode  node_;

  // goal information
  fawkes::NavGraphNode  goal_;

  fawkes::NavGraph *map_graph_;

  fawkes::NavGraphConstraintRepo *constraint_repo_;

  size_t key_;

  navgraph::EstimateFunction estimate_func_;
  navgraph::CostFunction     cost_func_;
};


} // end of namespace fawkes

#endif
