
/***************************************************************************
 *  navgraph.cpp - Topological graph
 *
 *  Created: Fri Sep 21 15:55:49 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <navgraph/navgraph.h>
#include <navgraph/constraints/constraint_repo.h>
#include <navgraph/search_state.h>
#include <core/exception.h>
#include <utils/search/astar.h>

#include <algorithm>
#include <list>
#include <set>
#include <queue>
#include <cmath>
#include <cstdio>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraph <navgraph/navgraph.h>
 * Topological map graph.
 * This class represents a topological graph using 2D map coordinates
 * with nodes and edges. Both can be annotated with certain free-form
 * properties which can be used at run-time for example to instruct
 * the robot behavior.
 *
 * The class supports change listeners. These are called whenever the graph
 * is changed, that is if a node or edge is added or if the graph is assigned
 * from another one (i.e. graph := new_graph).
 *
 * This class is based on KBSG RCSoft's MapGraph but has been
 * abstracted and improved.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param graph_name Name of the graph, for example to handle multiple
 * graphs, e.g. for multiple levels of a building.
 */
NavGraph::NavGraph(std::string graph_name)
{
  graph_name_ = graph_name;
  constraint_repo_ = LockPtr<NavGraphConstraintRepo>(new NavGraphConstraintRepo(),
						     /* recursive mutex */ true);
  search_default_funcs_ = true;
  search_estimate_func_ = NavGraphSearchState::straight_line_estimate;
  search_cost_func_     = NavGraphSearchState::euclidean_cost;
}


/** Virtual empty destructor. */
NavGraph::~NavGraph()
{
}


/** Assign/copy structures from another graph.
 * This method will remove internal data like nodes, and edges
 * and copy the data from the passed instance. The change listeners will
 * not be copied. The assignment operator will trigger all registered
 * change listeners to be called.
 * @param g graph from which to copy the data
 * @return reference to this instance
 */
NavGraph &
NavGraph::operator=(const NavGraph &g)
{
  graph_name_ = g.graph_name_;
  nodes_.clear();
  nodes_      = g.nodes_;
  edges_.clear();
  edges_      = g.edges_;

  notify_of_change();

  return *this;
}

/** Get graph name.
 * @return graph name
 */
std::string
NavGraph::name() const
{
  return graph_name_;
}


/** Get nodes of the graph.
 * @return const reference to vector of nodes of this graph
 */
const std::vector<NavGraphNode> &
NavGraph::nodes() const
{
  return nodes_;
}


/** Get edges of the graph.
 * @return const reference to vector of edges of this graph
 */
const std::vector<NavGraphEdge> &
NavGraph::edges() const
{
  return edges_;
}


/** Get locked pointer to constraint repository.
 * @return locked pointer to navgraph constraint repo. Note that you must
 * lock it when invoking operations on the repo.
 */
fawkes::LockPtr<NavGraphConstraintRepo>
NavGraph::constraint_repo() const
{
  return constraint_repo_;
}

/** Get a specified node.
 * @param name name of the node to get
 * @return the node representation of the searched node, if not
 * found returns an invalid node.
 */
NavGraphNode
NavGraph::node(std::string name) const
{
  std::vector<NavGraphNode>::const_iterator i;
  for (i = nodes_.begin(); i != nodes_.end(); ++i) {
    if (i->name() == name)  return *i;
  }
  return NavGraphNode();
}


/** Get node closest to a specified point with a certain property.
 * This search does *NOT* consider unconnected nodes.
 * @param pos_x X coordinate in global (map) frame
 * @param pos_y X coordinate in global (map) frame
 * @param property property the node must have to be considered,
 * empty string to not check for any property
 * @return node closest to the given point in the global frame, or an
 * invalid node if such a node cannot be found
 */
NavGraphNode
NavGraph::closest_node(float pos_x, float pos_y, std::string property)
{
  return closest_node(pos_x, pos_y, false, property);
}


/** Get node closest to a specified point with a certain property.
 * This search *does* consider unconnected nodes.
 * @param pos_x X coordinate in global (map) frame
 * @param pos_y X coordinate in global (map) frame
 * @param property property the node must have to be considered,
 * empty string to not check for any property
 * @return node closest to the given point in the global frame, or an
 * invalid node if such a node cannot be found
 */
NavGraphNode
NavGraph::closest_node_with_unconnected(float pos_x, float pos_y,
						   std::string property)
{
  return closest_node(pos_x, pos_y, true, property);
}

/** Get node closest to another node with a certain property.
 * This search does *NOT* consider unconnected nodes.
 * @param node_name the name of the node from which to start
 * @param property property the node must have to be considered,
 * empty string to not check for any property
 * @return node closest to the given point in the global frame, or an
 * invalid node if such a node cannot be found. The node will obviously
 * not be the node with the name @p node_name.
 */
NavGraphNode
NavGraph::closest_node_to(std::string node_name,
						      std::string property)
{
  return closest_node_to(node_name, false, property);
}

/** Get node closest to another node with a certain property.
 * This search *does* consider unconnected nodes.
 * @param node_name the name of the node from which to start
 * @param property property the node must have to be considered,
 * empty string to not check for any property
 * @return node closest to the given point in the global frame, or an
 * invalid node if such a node cannot be found. The node will obviously
 * not be the node with the name @p node_name.
 */
NavGraphNode
NavGraph::closest_node_to_with_unconnected(std::string node_name,
						      std::string property)
{
  return closest_node_to(node_name, true, property);
}

/** Get node closest to a specified point with a certain property.
 * @param pos_x X coordinate in global (map) frame
 * @param pos_y X coordinate in global (map) frame
 * @param consider_unconnected consider unconnected node for the search
 * of the closest node
 * @param property property the node must have to be considered,
 * empty string to not check for any property
 * @return node closest to the given point in the global frame, or an
 * invalid node if such a node cannot be found
 */
NavGraphNode
NavGraph::closest_node(float pos_x, float pos_y, bool consider_unconnected,
                                  std::string property)
{
  std::vector<NavGraphNode> nodes = search_nodes(property);

  float min_dist = HUGE;

  std::vector<NavGraphNode>::iterator i;
  std::vector<NavGraphNode>::iterator elem = nodes.begin();
  for (i = nodes.begin(); i != nodes.end(); ++i) {
    if (! consider_unconnected && i->unconnected())  continue;

    float dx   = i->x() - pos_x;
    float dy   = i->y() - pos_y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (sqrtf(dx * dx + dy * dy) < min_dist) {
      min_dist = dist;
      elem = i;
    }
  }

  if (elem == nodes.end()) {
    return NavGraphNode();
  } else {
    return *elem;
  }
}

/** Get node closest to another node with a certain property.
 * @param node_name the name of the node from which to start
 * @param consider_unconnected consider unconnected node for the search
 * of the closest node
 * @param property property the node must have to be considered,
 * empty string to not check for any property
 * @return node closest to the given point in the global frame, or an
 * invalid node if such a node cannot be found. The node will obviously
 * not be the node with the name @p node_name.
 */
NavGraphNode
NavGraph::closest_node_to(std::string node_name, bool consider_unconnected,
				     std::string property)
{
  NavGraphNode n = node(node_name);
  std::vector<NavGraphNode> nodes = search_nodes(property);

  float min_dist = HUGE;

  std::vector<NavGraphNode>::iterator i;
  std::vector<NavGraphNode>::iterator elem = nodes.begin();
  for (i = nodes.begin(); i != nodes.end(); ++i) {
    if (! consider_unconnected && i->unconnected())  continue;

    float dx   = i->x() - n.x();
    float dy   = i->y() - n.y();
    float dist = sqrtf(dx * dx + dy * dy);
    if ((sqrtf(dx * dx + dy * dy) < min_dist) && (i->name() != node_name)) {
      min_dist = dist;
      elem = i;
    }
  }

  if (elem == nodes.end()) {
    return NavGraphNode();
  } else {
    return *elem;
  }
}


/** Check if a certain node exists.
 * @param name name of the node to look for
 * @return true if a node with the given name exists, false otherwise
 */
bool
NavGraph::node_exists(std::string name) const
{
  std::vector<NavGraphNode>::const_iterator i;
  for (i = nodes_.begin(); i != nodes_.end(); ++i) {
    if (i->name() == name)  return true;
  }
  return false;
}

/** Search nodes for given property.
 * @param property property name to look for
 * @return vector of nodes having the specified property
 */
std::vector<NavGraphNode>
NavGraph::search_nodes(std::string property)
{
  if (property == "") {
    return nodes();
  } else {
    std::vector<NavGraphNode> rv;

    std::vector<NavGraphNode>::iterator i;
    for (i = nodes_.begin(); i != nodes_.end(); ++i) {
      if ( i->has_property(property) )  rv.push_back(*i);
    }

    return rv;
  }
}


/** Add a node.
 * @param node node to add
 */
void
NavGraph::add_node(NavGraphNode node)
{
  nodes_.push_back(node);
  notify_of_change();
}

/** Add an edge
 * @param edge edge to add
 */
void
NavGraph::add_edge(NavGraphEdge edge)
{
  edges_.push_back(edge);
  notify_of_change();
}


/** Get all default properties.
 * @return property map
 */
const std::map<std::string, std::string> &
NavGraph::default_properties() const
{
  return default_properties_;
}

/** Check if graph has specified default property.
 * @param property property key
 * @return true if node has specified property, false otherwise
 */
bool
NavGraph::has_default_property(std::string property) const
{
  return default_properties_.find(property) != default_properties_.end();
}

/** Get specified default property as string.
 * @param prop property key
 * @return default property value as string
 */
std::string
NavGraph::default_property(std::string prop) const
{
  std::map<std::string, std::string>::const_iterator p;
  if ((p = default_properties_.find(prop)) != default_properties_.end()) {
    return p->second;
  } else {
    return "";
  }
}

/** Get property converted to float.
 * @param prop property key
 * @return property value
 */
float
NavGraph::default_property_as_float(std::string prop) const
{
  return StringConversions::to_float(default_property(prop));
}

/** Get property converted to int.
 * @param prop property key
 * @return property value
 */
int
NavGraph::default_property_as_int(std::string prop) const
{
  return StringConversions::to_int(default_property(prop));
}

/** Get property converted to bol.
 * @param prop property key
 * @return property value
 */
bool
NavGraph::default_property_as_bool(std::string prop) const
{
  return StringConversions::to_bool(default_property(prop));
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraph::set_default_property(std::string property, std::string value)
{
  default_properties_[property] = value;
}

/** Set default properties.
 * This overwrites all existing properties.
 * @param properties map of property name to value as string
 */
void
NavGraph::set_default_properties(std::map<std::string, std::string> &properties)
{
  default_properties_ = properties;
}


/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraph::set_default_property(std::string property, float value)
{
  default_properties_[property] = StringConversions::to_string(value);
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraph::set_default_property(std::string property, int value)
{
  default_properties_[property] = StringConversions::to_string(value);
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraph::set_default_property(std::string property, bool value)
{
  default_properties_[property] = value ? "true" : "false";
}



/** Get nodes reachable from specified nodes.
 * @param node_name name of the node to get reachable nodes for
 * @return vector of names of nodes reachable from the specified node
 */
std::vector<std::string>
NavGraph::reachable_nodes(std::string node_name) const
{
  std::vector<std::string> rv;

  NavGraphNode n = node(node_name);
  if (! n.is_valid())  return rv;

  std::vector<NavGraphEdge>::const_iterator i;
  for (i = edges_.begin(); i != edges_.end(); ++i) {
    if (i->is_directed()) {
      if (i->from() == node_name) {
        rv.push_back(i->to());
      }
    } else {
      if (i->from() == node_name) {
        rv.push_back(i->to());
      } else if (i->to() == node_name) {
        rv.push_back(i->from());
      }
    }
  }

  std::sort(rv.begin(), rv.end());
  std::unique(rv.begin(), rv.end());
  return rv;
}


/** Set cost and cost estimation function for searching paths.
 * Note that this will influence each and every search (unless
 * custom functions are passed for the search). So use with caution.
 * We recommend to encapsulate different search modes as a plugin
 * that can be loaded to enable to new search functions.
 * Make sure to call unset_search_funcs() to restore the defaults.
 * The function points must obviously be valid for the whole lifetime
 * of the NavGraph or until unset.
 * @param estimate_func cost estimation function
 * @param cost_func actual cost function
 * @see NavGraph::search_path
 * @throw Exception if search functions have already been set.
 */
void
NavGraph::set_search_funcs(navgraph::EstimateFunction estimate_func,
			   navgraph::CostFunction     cost_func)
{
  if (! search_default_funcs_) {
    throw Exception("Custom actual and estimated cost functions have already been set");
  }
  search_default_funcs_ = false;
  search_estimate_func_ = estimate_func;
  search_cost_func_     = cost_func;
}


/** Reset actual and estimated cost function to defaults. */
void
NavGraph::unset_search_funcs()
{
  search_default_funcs_ = true;
  search_estimate_func_ = NavGraphSearchState::straight_line_estimate;
  search_cost_func_     = NavGraphSearchState::euclidean_cost;
}

/** Search for a path between two nodes with default distance costs.
 * This function executes an A* search to find an (optimal) path
 * from node @p from to node @p to.
 * By default (unless set otherwise, confirm using uses_default_search()),
 * the cost and estimated costs are calculated as the spatial euclidean
 * distance between nodes. The cost is the sum of costs of all edges
 * along the way from one node to another. The estimate is the straight line
 * distance from any given node to the goal node (which is provably admissible).
 * @param from node to search from
 * @param to goal node
 * @param use_constraints true to respect constraints imposed by the constraint
 * repository, false to ignore the repository searching as if there were no
 * constraints whatsoever.
 * @param compute_constraints if true re-compute constraints, otherwise use constraints
 * as-is, for example if they have been computed before to check for changes.
 * @return ordered vector of nodes which denote a path from @p from to @p to.
 * Note that the vector is empty if no path could be found (i.e. there is non
 * or it was prohibited when using constraints.
 */
fawkes::NavGraphPath
NavGraph::search_path(const NavGraphNode &from, const NavGraphNode &to,
		      bool use_constraints, bool compute_constraints)
{
  return search_path(from, to,
		     search_estimate_func_, search_cost_func_,
		     use_constraints, compute_constraints);
}

/** Search for a path between two nodes with default distance costs.
 * This function executes an A* search to find an (optimal) path
 * from node @p from to node @p to.
 * By default (unless set otherwise, confirm using uses_default_search()),
 * the cost and estimated costs are calculated as the spatial euclidean
 * distance between nodes. The cost is the sum of costs of all edges
 * along the way from one node to another. The estimate is the straight line
 * distance from any given node to the goal node (which is provably admissible).
 * @param from name of node to search from
 * @param to name of the goal node
 * @param use_constraints true to respect constraints imposed by the constraint
 * repository, false to ignore the repository searching as if there were no
 * constraints whatsoever.
 * @param compute_constraints if true re-compute constraints, otherwise use constraints
 * as-is, for example if they have been computed before to check for changes.
 * @return ordered vector of nodes which denote a path from @p from to @p to.
 * Note that the vector is empty if no path could be found (i.e. there is non
 * or it was prohibited when using constraints.
 */
fawkes::NavGraphPath
NavGraph::search_path(const std::string &from, const std::string &to,
		      bool use_constraints, bool compute_constraints)
{
  return search_path(from, to,
		     search_estimate_func_, search_cost_func_,
		     use_constraints, compute_constraints);
}

/** Search for a path between two nodes.
 * This function executes an A* search to find an (optimal) path
 * from node @p from to node @p to.
 * @param from name of node to search from
 * @param to name of the goal node
 * @param estimate_func function to estimate the cost from any node to the goal.
 * Note that the estimate function must be admissible for optimal A* search. That
 * means that for no query may the calculated estimate be higher than the actual
 * cost.
 * @param cost_func function to calculate the cost from a node to another adjacent
 * node. Note that the cost function is directly related to the estimate function.
 * For example, the cost can be calculated in terms of distance between nodes, or in
 * time that it takes to travel from one node to the other. The estimate function must
 * match the cost function to be admissible.
 * @param use_constraints true to respect constraints imposed by the constraint
 * repository, false to ignore the repository searching as if there were no
 * constraints whatsoever.
 * @param compute_constraints if true re-compute constraints, otherwise use constraints
 * as-is, for example if they have been computed before to check for changes.
 * @return ordered vector of nodes which denote a path from @p from to @p to.
 * Note that the vector is empty if no path could be found (i.e. there is non
 * or it was prohibited when using constraints.
 */
fawkes::NavGraphPath
NavGraph::search_path(const std::string &from, const std::string &to,
		      navgraph::EstimateFunction estimate_func,
		      navgraph::CostFunction cost_func,
		      bool use_constraints, bool compute_constraints)
{
  NavGraphNode from_node(node(from));
  NavGraphNode to_node(node(to));
  return search_path(from_node, to_node, estimate_func, cost_func,
		     use_constraints, compute_constraints);
}

/** Search for a path between two nodes.
 * This function executes an A* search to find an (optimal) path
 * from node @p from to node @p to.
 * @param from node to search from
 * @param to goal node
 * @param estimate_func function to estimate the cost from any node to the goal.
 * Note that the estimate function must be admissible for optimal A* search. That
 * means that for no query may the calculated estimate be higher than the actual
 * cost.
 * @param cost_func function to calculate the cost from a node to another adjacent
 * node. Note that the cost function is directly related to the estimate function.
 * For example, the cost can be calculated in terms of distance between nodes, or in
 * time that it takes to travel from one node to the other. The estimate function must
 * match the cost function to be admissible.
 * @param use_constraints true to respect constraints imposed by the constraint
 * repository, false to ignore the repository searching as if there were no
 * constraints whatsoever.
 * @param compute_constraints if true re-compute constraints, otherwise use constraints
 * as-is, for example if they have been computed before to check for changes.
 * @return ordered vector of nodes which denote a path from @p from to @p to.
 * Note that the vector is empty if no path could be found (i.e. there is non
 * or it was prohibited when using constraints.
 */
fawkes::NavGraphPath
NavGraph::search_path(const NavGraphNode &from, const NavGraphNode &to,
		      navgraph::EstimateFunction estimate_func,
		      navgraph::CostFunction cost_func,
		      bool use_constraints, bool compute_constraints)
{
  AStar astar;

  std::vector<AStarState *> a_star_solution;

  if (use_constraints) {
    constraint_repo_.lock();
    if (compute_constraints && constraint_repo_->has_constraints()) {
      constraint_repo_->compute();
    }

    NavGraphSearchState *initial_state =
      new NavGraphSearchState(from, to, this, estimate_func, cost_func,
			      *constraint_repo_);
    a_star_solution =  astar.solve(initial_state);
    constraint_repo_.unlock();
  } else {
    NavGraphSearchState *initial_state =
      new NavGraphSearchState(from, to, this, estimate_func, cost_func);
    a_star_solution =  astar.solve(initial_state);
  }

  std::vector<fawkes::NavGraphNode> path(a_star_solution.size());
  NavGraphSearchState *solstate;
  for (unsigned int i = 0; i < a_star_solution.size(); ++i ) {
    solstate = dynamic_cast<NavGraphSearchState *>(a_star_solution[i]);
    path[i] = solstate->node();
  }

  float cost =
    (! a_star_solution.empty())
      ? a_star_solution[a_star_solution.size() - 1]->total_estimated_cost
      : -1;

  return NavGraphPath(this, path, cost);
}


/** Calculate cost between two adjacent nodes.
 * It is not verified whether the nodes are actually adjacent, but the cost
 * function is simply applied. This is done to increase performance.
 * The calculation will use the currently registered cost function.
 * @param from first node
 * @param to second node
 * @return cost from @p from to @p to
 */
float
NavGraph::cost(const NavGraphNode &from, const NavGraphNode &to) const
{
  return search_cost_func_(from, to);
}


/** Make sure each node exists only once. */
void
NavGraph::assert_unique_nodes()
{
  std::list<std::string> names;
  std::vector<NavGraphNode>::iterator i;
  for (i = nodes_.begin(); i != nodes_.end(); ++i) {
    names.push_back(i->name());
  }
  names.sort();
  std::list<std::string>::iterator n;
  std::string last_name = "";
  for (n = names.begin(); n != names.end(); ++n) {
    if (*n == last_name) {
      throw Exception("Node '%s' exists at least twice", last_name.c_str());
    }
    last_name = *n;
  }
}

/** Make sure each edge exists only once. */
void
NavGraph::assert_unique_edges()
{
  for (size_t i = 0; i < edges_.size(); ++i) {
    for (size_t j = i+1; j < edges_.size(); ++j) {
      if (edges_[i].from() == edges_[j].from() &&
          edges_[i].to() == edges_[j].to())
      {
        throw Exception("Edge '%s - %s' is defined twice",
                        edges_[i].from().c_str(), edges_[i].to().c_str());
      }
      if (edges_[i].from() == edges_[j].to() &&
          edges_[i].to() == edges_[j].from() &&
          (!edges_[i].is_directed() || !edges_[j].is_directed()))
      {
        throw Exception("Edge '%s - %s' and '%s - %s' both exist "
                        "and at least one is not directed",
                        edges_[i].from().c_str(), edges_[i].to().c_str(),
                        edges_[j].from().c_str(), edges_[j].to().c_str());
      }
    }
  }
}

/** Make sure each node in the edges exists. */
void
NavGraph::assert_valid_edges()
{
  for (size_t i = 0; i < edges_.size(); ++i) {
    if (! node_exists(edges_[i].from())) {
      throw Exception("Node '%s' for edge '%s' -> '%s' does not exist",
                      edges_[i].from().c_str(), edges_[i].from().c_str(),
                      edges_[i].to().c_str());
    }

    if (! node_exists(edges_[i].to())) {
      throw Exception("Node '%s' for edge '%s' -> '%s' does not exist",
                      edges_[i].to().c_str(), edges_[i].from().c_str(),
                      edges_[i].to().c_str());
    }
  }
}



void
NavGraph::assert_connected()
{
  std::set<std::string> traversed;
  std::set<std::string> nodeset;
  std::queue<NavGraphNode> q;
  q.push(nodes_.front());

  while (! q.empty()) {
    NavGraphNode &n = q.front();
    traversed.insert(n.name());

    const std::vector<std::string> & reachable = n.reachable_nodes();

    if (n.unconnected() && !reachable.empty()) {
      throw Exception("Node %s is marked unconnected but nodes are reachable from it",
		      n.name().c_str());
    }
    std::vector<std::string>::const_iterator r;
    for (r = reachable.begin(); r != reachable.end(); ++r) {
      NavGraphNode target(node(*r));
      if (target.unconnected()) {
	throw Exception("Node %s is marked unconnected but is reachable from node %s\n",
			target.name().c_str(), n.name().c_str());
      }
      if (traversed.find(*r) == traversed.end()) q.push(node(*r));
    }
    q.pop();
  }

  std::vector<NavGraphNode>::iterator n;
  for (n = nodes_.begin(); n != nodes_.end(); ++n) {
    nodeset.insert(n->name());
  }

  if (traversed.size() != nodeset.size()) {
    std::set<std::string> nodediff;
    std::set_difference(nodeset.begin(), nodeset.end(),
			traversed.begin(), traversed.end(),
			std::inserter(nodediff, nodediff.begin()));

    // the nodes might be unconnected, in which case it is not
    // an error that they were mentioned. But it might still be
    // a problem if there was a *directed* outgoing edge from the
    // unconnected node, which we couldn't have spotted earlier
    std::set<std::string>::const_iterator ud = nodediff.begin();
    while (ud != nodediff.end()) {
      NavGraphNode udnode(node(*ud));
      if (udnode.unconnected()) {
	// it's ok to be in the disconnected set, but check if it has any
	// reachable nodes which is forbidden
	if (! udnode.reachable_nodes().empty()) {
	  throw Exception("Node %s is marked unconnected but nodes are reachable from it",
			  ud->c_str());
	}
#if __cplusplus > 201100L || defined(__GXX_EXPERIMENTAL_CXX0X__)
	ud = nodediff.erase(ud);
#else
	std::set<std::string>::const_iterator delit = ud;
	++ud;
	nodediff.erase(delit);
#endif
      } else {
	++ud;
      }
    }

    if (! nodediff.empty()) {
      std::set<std::string>::iterator d = nodediff.begin();
      std::string disconnected = *d;
      for (++d; d != nodediff.end(); ++d) {
	disconnected += ", " + *d;
      }
      throw Exception("The graph is not fully connected, "
		      "cannot reach (%s) from '%s' for example",
		      disconnected.c_str(), nodes_[0].name().c_str());
    }
  }
}



/** Calculate eachability relations.
 * This will set the directly reachable nodes on each
 * of the graph nodes. 
 */
void
NavGraph::calc_reachability()
{
  if (nodes_.empty())  return;

  assert_unique_nodes();
  assert_unique_edges();
  assert_valid_edges();
  std::vector<NavGraphNode>::iterator i;
  for (i = nodes_.begin(); i != nodes_.end(); ++i) {
    i->set_reachable_nodes(reachable_nodes(i->name()));
  }
  assert_connected();

  std::vector<NavGraphEdge>::iterator e;
  for (e = edges_.begin(); e != edges_.end(); ++e) {
    e->set_nodes(node(e->from()), node(e->to()));
  }
}


/** Add a change listener.
 * @param listener listener to add
 */
void
NavGraph::add_change_listener(ChangeListener *listener)
{
  change_listeners_.push_back(listener);
}

/** Remove a change listener.
 * @param listener listener to remove
 */
void
NavGraph::remove_change_listener(ChangeListener *listener)
{
  change_listeners_.remove(listener);
}

/** Notify all listeners of a change. */
void
NavGraph::notify_of_change() throw()
{
  std::list<ChangeListener *> tmp_listeners = change_listeners_;

  std::list<ChangeListener *>::iterator l;
  for (l = tmp_listeners.begin(); l != tmp_listeners.end(); ++l) {
    (*l)->graph_changed();
  }
}

/** @class NavGraph::ChangeListener <navgraph/navgraph.h>
 * Topological graph change listener.
 * @author Tim Niemueller
 *
 * @fn void NavGraph::ChangeListener::graph_changed() throw() = 0
 * Function called if the graph has been changed.
 */

/** Virtual empty destructor. */
NavGraph::ChangeListener::~ChangeListener()
{
}


} // end of namespace fawkes
