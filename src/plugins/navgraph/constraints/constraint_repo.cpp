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
#include <plugins/navgraph/constraints/constraint_repo.h>

#include <logging/logger.h>
#include <algorithm>

using namespace std;

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ConstraintRepo <plugins/navgraph/constraints/constraint_repo.h>
 * Constraint repository to maintain blocks on nodes.
 * @author Sebastian Reuter
 * @author Tim Niemueller
 */


/** Constructor.
 * @param logger logger used for debug output
 */
ConstraintRepo::ConstraintRepo(Logger *logger)
{
  logger_ = logger;
  modified_ = false;
}

/** Destructor. */
ConstraintRepo::~ConstraintRepo()
{
}


/** Register a constraint.
 * @param constraint node constraint to register
 */
void
ConstraintRepo::register_constraint(NavGraphNodeConstraint* constraint)
{
  modified_ = true;
  node_constraints_.push_back(constraint);
}

/** Register a constraint.
 * @param constraint edge constraint to register
 */
void
ConstraintRepo::register_constraint(NavGraphEdgeConstraint* constraint)
{
  modified_ = true;
  edge_constraints_.push_back(constraint);
}


/** Unregister a constraint by name.
 * @param name name of constraint to remove.
 */
void
ConstraintRepo::unregister_constraint(std::string name)
{
  modified_ = true;

  NodeConstraintList::iterator nc =
    std::find_if(node_constraints_.begin(), node_constraints_.end(),
		 [&name](const NavGraphNodeConstraint *c) {
		   return *c == name;
		 });
  if (nc != node_constraints_.end()) {
    logger_->log_debug("ConstraintRepo", "Unregistering node constraint %s",
		       (*nc)->name().c_str());
    node_constraints_.erase(nc);
  }

  EdgeConstraintList::iterator ec =
    std::find_if(edge_constraints_.begin(), edge_constraints_.end(),
		 [&name](const NavGraphEdgeConstraint *c) {
		   return *c == name;
		 });
  if (ec != edge_constraints_.end()) {
    logger_->log_debug("ConstraintRepo", "Unregistering edge constraint %s",
		       (*ec)->name().c_str());
    edge_constraints_.erase(ec);
  }
}


/** Check by name if a constraint has been registered.
 * @param name name of constraint to look for
 * @return true if a constraint with the given name has been registered,
 * false otherwise
 */
bool
ConstraintRepo::has_constraint(std::string &name)
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

  return false;
}


/** Get a node constraint by name.
 * @param name name of constraint to retrieve
 * @return if found returns a pointer to the node constraint, NULL if not found
 */
fawkes::NavGraphNodeConstraint *
ConstraintRepo::get_node_constraint(std::string &name)
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
ConstraintRepo::get_edge_constraint(std::string &name)
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


/** Get a list of registered node constraints.
 * @return list of node constraints
 */
const ConstraintRepo::NodeConstraintList &
ConstraintRepo::node_constraints() const
{
  return node_constraints_;
}


/** Get a list of registered edge constraints.
 * @return list of edge constraints
 */
const ConstraintRepo::EdgeConstraintList &
ConstraintRepo::edge_constraints() const
{
  return edge_constraints_;
}


/** Check if there are any constraints at all.
 * @return true if constraints have been registered, false otherwise
 */
bool
ConstraintRepo::has_constraints() const
{
  return (! (node_constraints_.empty() && edge_constraints_.empty()));
}


/** Call compute method on all registered constraints. */
void
ConstraintRepo::compute()
{
  modified_ = true;
  for (fawkes::NavGraphNodeConstraint *c : node_constraints_) {
    c->compute();
  }
  for (fawkes::NavGraphEdgeConstraint *c : edge_constraints_) {
    c->compute();
  }
}


/** Check if any constraint in the repo blocks the node.
 * @param node Node to check for a block
 * @return the (first) node constraint that blocked the node,
 * NULL if the node is not blocked
 */
fawkes::NavGraphNodeConstraint *
ConstraintRepo::blocks(const fawkes::TopologicalMapNode &node)
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
ConstraintRepo::blocks(const std::vector<fawkes::TopologicalMapNode> &nodes)
{
  std::map<std::string, std::string> rv;
  for (const fawkes::TopologicalMapNode &n : nodes) {
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
ConstraintRepo::blocks(const fawkes::TopologicalMapNode &from,
		       const fawkes::TopologicalMapNode &to)
{
  for (fawkes::NavGraphEdgeConstraint *c : edge_constraints_) {
    if (c->blocks(from, to)) {
      return c;
    }
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
ConstraintRepo::blocks(const std::vector<fawkes::TopologicalMapEdge> &edges)
{
  std::map<std::pair<std::string, std::string>, std::string> rv;
  for (const fawkes::TopologicalMapEdge &e : edges) {
    for (fawkes::NavGraphEdgeConstraint *c : edge_constraints_) {
      if (c->blocks(e.from_node(), e.to_node()) ){
	rv[std::make_pair(e.from(), e.to())] = c->name();
      }
    }
  }

  return rv;
}


/** Check if the constraint repo has been modified.
 * @param reset_modified true to reset the modified flag, false to leave it
 * @return true if the constraint repo has been modified, false otherwise
 */
bool
ConstraintRepo::modified(bool reset_modified)
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
