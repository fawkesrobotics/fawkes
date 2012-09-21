/***************************************************************************
 *  search_state.cpp - Graph-based global path planning - A-Star search state
 *
 *  Created: Tue Sep 18 18:21:56 2012
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

#include "search_state.h"

#include <functional>
#include <cmath>

using namespace fawkes;

/** @class NavGraphSearchState "search_state.h"
 * Graph-based path planner A* search state.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param node graph node this search state represents
 * @param goal graph node of the goal
 * @param new_cost the cost until to this node from the start
 * @param parent parent search state
 * @param map_graph map graph
 */
NavGraphSearchState::NavGraphSearchState(TopologicalMapNode node, TopologicalMapNode goal,
					 double new_cost, NavGraphSearchState * parent,
					 TopologicalMapGraph *map_graph)
{
  node_ = node;
  goal_ = goal;
  map_graph_ = map_graph;

  past_cost = new_cost;
  this->parent = parent;
  total_estimated_cost = past_cost + estimate();

  std::hash<std::string> h;
  key_ = h(node_.name());
}


/** Destructor. */
NavGraphSearchState::~NavGraphSearchState()
{
}


/** Get graph node corresponding to this search state.
 * @return graph node corresponding to this search state
 */
fawkes::TopologicalMapNode &
NavGraphSearchState::node()
{
  return node_;
}


double
NavGraphSearchState::estimate()
{
  return sqrtf(powf(node_.x() - goal_.x(), 2) +
               powf(node_.y() - goal_.y(), 2));
}


bool
NavGraphSearchState::is_goal()
{
  return (node_.name() == goal_.name());
}


std::vector<AStarState *>
NavGraphSearchState::children()
{
  double distance = -1.0;
  std::vector< AStarState * > children;
  children.clear();

  std::vector<std::string> descendants = node_.reachable_nodes();

  for (unsigned int i = 0; i < descendants.size(); ++i) {
    TopologicalMapNode d = map_graph_->node(descendants[i]);
    distance = sqrt(pow(node_.x() - d.x(), 2) +
		    pow(node_.y() - d.y(), 2) );
    children.push_back(new NavGraphSearchState(d, goal_, 
					       past_cost + distance, this,
					       map_graph_) );
  }

  return children;
}

