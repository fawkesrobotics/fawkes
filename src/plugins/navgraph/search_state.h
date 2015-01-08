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
#include <plugins/navgraph/constraints/constraint_repo.h>
#include <navgraph/navgraph.h>
#include <core/utils/lockptr.h>

#include <cmath>

class NavGraphSearchState : public fawkes::AStarState
{
public:
  NavGraphSearchState(fawkes::NavGraphNode node, fawkes::NavGraphNode goal,
		      fawkes::NavGraph *map_graph,
		      fawkes::ConstraintRepo *constraint_repo = NULL);

  NavGraphSearchState(fawkes::NavGraphNode node, fawkes::NavGraphNode goal,
		      double cost_sofar, NavGraphSearchState *parent_state,
		      fawkes::NavGraph *map_graph,
		      fawkes::ConstraintRepo *constraint_repo = NULL);
  ~NavGraphSearchState();

  fawkes::NavGraphNode & node();

  virtual size_t key() { return key_; }
  virtual float  estimate();
  virtual bool   is_goal();

  /** Determine cost between two nodes.
   * @param from originating node
   * @param to destination node
   * @return cost from @p from to @p to.
   */
  static inline float
    cost(const fawkes::NavGraphNode &from,
	 const fawkes::NavGraphNode &to)
  {
    return sqrtf(powf(to.x() - from.x(), 2) +
		 powf(to.y() - from.y(), 2) );
  }

  /** Determine cost between the node of this search state and a given node.
   * @param d destination node
   * @return cost from this node to @p d.
   */
  inline float cost(const fawkes::NavGraphNode &d) {
    return cost(node_, d);
  }

 private:

  std::vector<AStarState *> children();

  // state information
  fawkes::NavGraphNode  node_;

  // goal information
  fawkes::NavGraphNode  goal_;

  fawkes::NavGraph *map_graph_;

  fawkes::ConstraintRepo *constraint_repo_;
  bool constrained_search_;

  size_t key_;
};



#endif
