/***************************************************************************
 *  constraint_repo.cpp - navgraph constraint repository
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
#include <navgraph/constraints/constraint_repo.h>

#include <algorithm>

using namespace std;

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphConstraintRepo <navgraph/constraints/constraint_repo.h>
 * Constraint repository to maintain blocks on nodes.
 * @author Sebastian Reuter
 * @author Tim Niemueller
 */


/** Constructor. */
NavGraphConstraintRepo::NavGraphConstraintRepo()
{
  modified_ = false;
}

/** Destructor. */
NavGraphConstraintRepo::~NavGraphConstraintRepo()
{
}


/** Register a constraint.
 * @param constraint node constraint to register
 */
void
NavGraphConstraintRepo::register_constraint(NavGraphNodeConstraint* constraint)
{
  modified_ = true;
  node_constraints_.push_back(constraint);
}

/** Register a constraint.
 * @param constraint edge constraint to register
 */
void
NavGraphConstraintRepo::register_constraint(NavGraphEdgeConstraint* constraint)
{
  modified_ = true;
  edge_constraints_.push_back(constraint);
}

/** Register an edge cost constraint.
 * @param constraint edge cost constraint to register
 */
void
NavGraphConstraintRepo::register_constraint(NavGraphEdgeCostConstraint* constraint)
{
  modified_ = true;
  edge_cost_constraints_.push_back(constraint);
}


/** Unregister a constraint by name.
 * @param name name of constraint to remove.
 */
void
NavGraphConstraintRepo::unregister_constraint(std::string name)
{
  modified_ = true;

  NodeConstraintList::iterator nc =
    std::find_if(node_constraints_.begin(), node_constraints_.end(),
		 [&name](const NavGraphNodeConstraint *c) {
		   return *c == name;
		 });
  if (nc != node_constraints_.end()) {
    node_constraints_.erase(nc);
  }

  EdgeConstraintList::iterator ec =
    std::find_if(edge_constraints_.begin(), edge_constraints_.end(),
		 [&name](const NavGraphEdgeConstraint *c) {
		   return *c == name;
		 });
  if (ec != edge_constraints_.end()) {
    edge_constraints_.erase(ec);
  }

  EdgeCostConstraintList::iterator ecc =
    std::find_if(edge_cost_constraints_.begin(), edge_cost_constraints_.end(),
		 [&name](const NavGraphEdgeCostConstraint *c) {
		   return *c == name;
		 });
  if (ecc != edge_cost_constraints_.end()) {
    edge_cost_constraints_.erase(ecc);
  }
}


/** Check by name if a constraint has been registered.
 * @param name name of constraint to look for
 * @return true if a constraint with the given name has been registered,
 * false otherwise
 */
bool
NavGraphConstraintRepo::has_constraint(std::string &name)
{
  NodeConstraintList::iterator nc =
    std::find_if(node_constraints_.begin(), node_constraints_.end(),
		 [&name](const NavGraphNodeConstraint *c) {
		   return *c == name;
		 });
  if (nc != node_constraints_.end()) return true;

  EdgeConstraintList::iterator ec =
    std::find_if(edge_constraints_.begin(), edge_constraints_.end(),
		 [&name](const NavGraphEdgeConstraint *c) {
		   return *c == name;
		 });
  if (ec != edge_constraints_.end()) return true;

  EdgeCostConstraintList::iterator ecc =
    std::find_if(edge_cost_constraints_.begin(), edge_cost_constraints_.end(),
		 [&name](const NavGraphEdgeCostConstraint *c) {
		   return *c == name;
		 });
  if (ecc != edge_cost_constraints_.end()) return true;

  return false;
}


/** Get a node constraint by name.
 * @param name name of constraint to retrieve
 * @return if found returns a pointer to the node constraint, NULL if not found
 */
fawkes::NavGraphNodeConstraint *
NavGraphConstraintRepo::get_node_constraint(std::string &name)
{
  NodeConstraintList::iterator it =
    std::find_if(node_constraints_.begin(), node_constraints_.end(),
		 [&name](const NavGraphNodeConstraint *c) {
		   return *c == name;
		 });
  if (it != node_constraints_.end()) {
    return *it;
  }

  return NULL;
}

/** Get an edge constraint by name.
 * @param name name of constraint to retrieve
 * @return if found returns a pointer to the edge constraint, NULL if not found
 */
fawkes::NavGraphEdgeConstraint *
NavGraphConstraintRepo::get_edge_constraint(std::string &name)
{
  EdgeConstraintList::iterator it =
    std::find_if(edge_constraints_.begin(), edge_constraints_.end(),
		 [&name](const NavGraphEdgeConstraint *c) {
		   return *c == name;
		 });
  if (it != edge_constraints_.end()) {
    return *it;
  }

  return NULL;
}


/** Get an edge cost constraint by name.
 * @param name name of constraint to retrieve
 * @return if found returns a pointer to the edge cost constraint, NULL if not found
 */
fawkes::NavGraphEdgeCostConstraint *
NavGraphConstraintRepo::get_edge_cost_constraint(std::string &name)
{
  EdgeCostConstraintList::iterator it =
    std::find_if(edge_cost_constraints_.begin(), edge_cost_constraints_.end(),
		 [&name](const NavGraphEdgeCostConstraint *c) {
		   return *c == name;
		 });
  if (it != edge_cost_constraints_.end()) {
    return *it;
  }

  return NULL;
}


/** Get a list of registered node constraints.
 * @return list of node constraints
 */
const NavGraphConstraintRepo::NodeConstraintList &
NavGraphConstraintRepo::node_constraints() const
{
  return node_constraints_;
}


/** Get a list of registered edge constraints.
 * @return list of edge constraints
 */
const NavGraphConstraintRepo::EdgeConstraintList &
NavGraphConstraintRepo::edge_constraints() const
{
  return edge_constraints_;
}

/** Get a list of registered edge cost constraints.
 * @return list of edge cost constraints
 */
const NavGraphConstraintRepo::EdgeCostConstraintList &
NavGraphConstraintRepo::edge_cost_constraints() const
{
  return edge_cost_constraints_;
}


/** Check if there are any constraints at all.
 * @return true if constraints have been registered, false otherwise
 */
bool
NavGraphConstraintRepo::has_constraints() const
{
  return (! (node_constraints_.empty() &&
	     edge_constraints_.empty() &&
	     edge_cost_constraints_.empty()));
}


/** Call compute method on all registered constraints.
 * @return true if any constraint reported a change, false otherwise
 */
bool
NavGraphConstraintRepo::compute()
{
  bool modified = false;
  for (fawkes::NavGraphNodeConstraint *c : node_constraints_) {
    if (c->compute())  modified = true;
  }
  for (fawkes::NavGraphEdgeConstraint *c : edge_constraints_) {
    if (c->compute())  modified = true;
  }
  for (fawkes::NavGraphEdgeCostConstraint *c : edge_cost_constraints_) {
    if (c->compute())  modified = true;
  }

  return modified;
}


/** Check if any constraint in the repo blocks the node.
 * @param node Node to check for a block
 * @return the (first) node constraint that blocked the node,
 * NULL if the node is not blocked
 */
fawkes::NavGraphNodeConstraint *
NavGraphConstraintRepo::blocks(const fawkes::NavGraphNode &node)
{
  for (fawkes::NavGraphNodeConstraint *c : node_constraints_) {
    if (c->blocks(node)) {
      return c;
    }
  }

  return NULL;
}


/** Check if any constraint in the repo blocks (some) nodes.
 * @param nodes vector of nodes to check for a block
 * @return map of blocked nodes, first element is the node name,
 * second element is the name of the constraint that blocks the node.
 * Nodes from @p nodes that are not blocked will not appear in the map.
 */
std::map<std::string, std::string>
NavGraphConstraintRepo::blocks(const std::vector<fawkes::NavGraphNode> &nodes)
{
  std::map<std::string, std::string> rv;
  for (const fawkes::NavGraphNode &n : nodes) {
    for (fawkes::NavGraphNodeConstraint *c : node_constraints_) {
      if (c->blocks(n)) {
	rv[n.name()] = c->name();
      }
    }
  }

  return rv;
}


/** Check if any constraint in the repo blocks the edge.
 * @param from node from which the edge originates
 * @param to node to which the edge leads
 * @return the (first) edge constraint that blocked the node,
 * NULL if the node is not blocked
 */
fawkes::NavGraphEdgeConstraint *
NavGraphConstraintRepo::blocks(const fawkes::NavGraphNode &from,
		       const fawkes::NavGraphNode &to)
{
  for (fawkes::NavGraphEdgeConstraint *c : edge_constraints_) {
    if (c->blocks(from, to)) {
      return c;
    }
  }

  return NULL;
}

/** Check if any constraint in the repo increases the cost of the edge.
 * @param from node from which the edge originates
 * @param to node to which the edge leads
 * @return the (first) edge cost constraint that increases the cost of
 * the node, i.e. that returns a cost factor >= 1.00001.
 */
fawkes::NavGraphEdgeCostConstraint *
NavGraphConstraintRepo::increases_cost(const fawkes::NavGraphNode &from,
			       const fawkes::NavGraphNode &to)
{
  for (fawkes::NavGraphEdgeCostConstraint *c : edge_cost_constraints_) {
    if (c->cost_factor(from, to) >= 1.00001) {
      return c;
    }
  }

  return NULL;
}


/** Check if any constraint in the repo increases the cost of the edge.
 * @param from node from which the edge originates
 * @param to node to which the edge leads
 * @param cost_factor upon return with a non-NULL edge cost constraints
 * contains the cost increase.
 * @return the edge cost constraint that returns the highest increase
 * in cost of the node (and by a cost factor of at least >= 1.00001).
 */
fawkes::NavGraphEdgeCostConstraint *
NavGraphConstraintRepo::increases_cost(const fawkes::NavGraphNode &from,
			       const fawkes::NavGraphNode &to,
			       float & cost_factor)
{
  float max_cost = 1.0;
  fawkes::NavGraphEdgeCostConstraint *max_c = NULL;
  for (fawkes::NavGraphEdgeCostConstraint *c : edge_cost_constraints_) {
    float cost_factor = c->cost_factor(from, to);
    if (cost_factor > max_cost) {
      max_cost = cost_factor;
      max_c    = c;
    }
  }
  if (max_cost >= 1.00001) {
    cost_factor = max_cost;
    return max_c;
  }

  return NULL;
}


/** Check if any constraint in the repo blocks (some) edges.
 * @param edges vector of edges to check for a block
 * @return map of blocked edges, first element is a pair of the node names,
 * second element is the name of the constraint that blocks the edge.
 * Edges from @p edges that are not blocked will not appear in the map.
 */
std::map<std::pair<std::string, std::string>, std::string>
NavGraphConstraintRepo::blocks(const std::vector<fawkes::NavGraphEdge> &edges)
{
  std::map<std::pair<std::string, std::string>, std::string> rv;
  for (const fawkes::NavGraphEdge &e : edges) {
    for (fawkes::NavGraphEdgeConstraint *c : edge_constraints_) {
      if (c->blocks(e.from_node(), e.to_node()) ){
	rv[std::make_pair(e.from(), e.to())] = c->name();
      }
    }
  }

  return rv;
}



/** Get the highest increasing cost factor for an edge.
 * This methods goes through all of the given edges and queries all
 * edge cost constraints. If any constraint increases the cost of an
 * edge (cost >= 1.00001), it adds a tuple of the start node name, end
 * node name, constraint name, and cost factor of the constraint that
 * returned the highest cost factor to the list.
 *
 * @param edges vector of edges to check for a block
 * @return tuple of edges with increased costs consisting of start node name,
 * target node name, name and cost factor of constraint returning the highest
 * cost increase factor.
 * Edges for which no increase has been indicated will not be returned in the
 * list of tuples.
 */
std::list<std::tuple<std::string, std::string, std::string, float>>
NavGraphConstraintRepo::cost_factor(const std::vector<fawkes::NavGraphEdge> &edges)
{
  std::list<std::tuple<std::string, std::string, std::string, float>> rv;
  for (const fawkes::NavGraphEdge &e : edges) {
    float max_cost = 1.0;
    fawkes::NavGraphEdgeCostConstraint *max_c = NULL;
    for (fawkes::NavGraphEdgeCostConstraint *c : edge_cost_constraints_) {
      float cost_factor = c->cost_factor(e.from_node(), e.to_node());
      if (cost_factor > max_cost) {
	max_cost = cost_factor;
	max_c    = c;
      }
    }
    if (max_c && max_cost >= 1.00001) {
      rv.push_back(std::make_tuple(e.from(), e.to(), max_c->name(), max_cost));
    }
  }

  return rv;
}


/** Get the highest increasing cost factor for an edge.
 * This methods goes through all of the given edges and queries all
 * edge cost constraints. If any constraint increases the cost of an
 * edge (cost >= 1.00001), it adds a tuple of the start node name, end
 * node name, constraint name, and cost factor of the constraint that
 * returned the highest cost factor to the list.
 *
 * @param from start node of the edge
 * @param to destination node of edge
 * @return highest cost factor denoted by any edge or 1.0 if no constraint
 * has been specified.
 */
float
NavGraphConstraintRepo::cost_factor(const fawkes::NavGraphNode &from,
			    const fawkes::NavGraphNode &to)
{
  float max_cost = 1.0;
  for (fawkes::NavGraphEdgeCostConstraint *c : edge_cost_constraints_) {
    float cost_factor = c->cost_factor(from, to);
    if (cost_factor > max_cost) {
      max_cost = cost_factor;
    }
  }

  return max_cost;
}


/** Check if the constraint repo has been modified.
 * @param reset_modified true to reset the modified flag, false to leave it
 * @return true if the constraint repo has been modified, false otherwise
 */
bool
NavGraphConstraintRepo::modified(bool reset_modified)
{
  if (reset_modified) {
    bool rv = modified_;
    modified_ =false;
    return rv;
  } else {
    return modified_;
  }
}

} // namespace
