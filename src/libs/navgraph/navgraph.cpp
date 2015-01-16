
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

#include <Eigen/Geometry>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

namespace navgraph {
  const char *PROP_ORIENTATION = "orientation";
} // end of namespace fawkes::navgraph


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
NavGraph::NavGraph(const std::string &graph_name)
{
  graph_name_ = graph_name;
  constraint_repo_ = LockPtr<NavGraphConstraintRepo>(new NavGraphConstraintRepo(),
						     /* recursive mutex */ true);
  search_default_funcs_ = true;
  search_estimate_func_ = NavGraphSearchState::straight_line_estimate;
  search_cost_func_     = NavGraphSearchState::euclidean_cost;
  reachability_calced_  = false;
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
NavGraph::node(const std::string &name) const
{
  std::vector<NavGraphNode>::const_iterator n =
    std::find_if(nodes_.begin(), nodes_.end(), 
		 [&name](const NavGraphNode &node) {
		   return node.name() == name;
		 });
  if (n != nodes_.end()) {
    return *n;
  } else {
    return NavGraphNode();
  }
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
NavGraph::closest_node(float pos_x, float pos_y, const std::string &property) const
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
					const std::string &property) const
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
NavGraph::closest_node_to(const std::string &node_name,
			  const std::string &property) const
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
NavGraph::closest_node_to_with_unconnected(const std::string &node_name,
					   const std::string &property) const
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
		       const std::string &property) const
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
NavGraph::closest_node_to(const std::string &node_name, bool consider_unconnected,
			  const std::string &property) const
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


/** Get a specified edge.
 * @param from originating node name
 * @param to target node name
 * @return the edge representation for the edge with the given
 * originating and target nodes or an invalid edge if the edge
 * cannot be found
 */
NavGraphEdge
NavGraph::edge(const std::string &from, const std::string &to) const
{
  std::vector<NavGraphEdge>::const_iterator e =
    std::find_if(edges_.begin(), edges_.end(), 
		 [&from, &to](const NavGraphEdge &edge) {
		   return (edge.from() == from && edge.to() == to) ||
		     (! edge.is_directed() && (edge.to() == from && edge.from() == to));
		 });
  if (e != edges_.end()) {
    return *e;
  } else {
    return NavGraphEdge();
  }
}


/** Get edge closest to a specified point.
 * The point must be within an imaginery line segment parallel to
 * the edge, that is a line perpendicular to the edge must go
 * through the point and a point on the edge line segment.
 * @param pos_x X coordinate in global (map) frame of point
 * @param pos_y X coordinate in global (map) frame of point
 * @return edge closest to the given point, or invalid edge if
 * such an edge does not exist.
 */
NavGraphEdge
NavGraph::closest_edge(float pos_x, float pos_y) const
{
  float min_dist = std::numeric_limits<float>::max();

  NavGraphEdge rv;

  Eigen::Vector2f point(pos_x, pos_y);
  for (const NavGraphEdge &edge : edges_) {
    const Eigen::Vector2f origin(edge.from_node().x(), edge.from_node().y());
    const Eigen::Vector2f target(edge.to_node().x(), edge.to_node().y());
    const Eigen::Vector2f direction(target - origin);
    const Eigen::Vector2f direction_norm = direction.normalized();
    const Eigen::Vector2f diff = point - origin;
    const float t = direction.dot(diff) / direction.squaredNorm();

    if (t >= 0.0 && t <= 1.0) {
      // projection of the point onto the edge is within the line segment
      float distance = (diff - direction_norm.dot(diff) * direction_norm).norm();
      if (distance < min_dist) {
	min_dist = distance;
	rv = edge;
      }
    }
  }

  return rv;
}

/** Search nodes for given property.
 * @param property property name to look for
 * @return vector of nodes having the specified property
 */
std::vector<NavGraphNode>
NavGraph::search_nodes(const std::string &property) const
{
  if (property == "") {
    return nodes();
  } else {
    std::vector<NavGraphNode> rv;

    std::vector<NavGraphNode>::const_iterator i;
    for (i = nodes_.begin(); i != nodes_.end(); ++i) {
      if ( i->has_property(property) )  rv.push_back(*i);
    }

    return rv;
  }
}


/** Check if a certain node exists.
 * @param node node to look for (will check for a node with the same name)
 * @return true if a node with the same name as the given node exists, false otherwise
 */
bool
NavGraph::node_exists(const NavGraphNode &node) const
{
  std::vector<NavGraphNode>::const_iterator n =
    std::find(nodes_.begin(), nodes_.end(), node);
  return (n != nodes_.end());
}


/** Check if a certain node exists.
 * @param name name of the node to look for
 * @return true if a node with the given name exists, false otherwise
 */
bool
NavGraph::node_exists(const std::string &name) const
{
  std::vector<NavGraphNode>::const_iterator n =
    std::find_if(nodes_.begin(), nodes_.end(),
		 [&name](const NavGraphNode &node) {
		   return node.name() == name;
		 });
  return (n != nodes_.end());
}

/** Check if a certain edge exists.
 * @param edge edge to look for (will check for a node with the same originating and target node)
 * @return true if an edge with the same originating and target node exists, false otherwise
 */
bool
NavGraph::edge_exists(const NavGraphEdge &edge) const
{
  std::vector<NavGraphEdge>::const_iterator e =
    std::find(edges_.begin(), edges_.end(), edge);
  return (e != edges_.end());
}

/** Check if a certain edge exists.
 * @param from originating node name
 * @param to target node name
 * @return true if an edge with the same originating and target node exists, false otherwise
 */
bool
NavGraph::edge_exists(const std::string &from, const std::string &to) const
{
  std::vector<NavGraphEdge>::const_iterator e =
    std::find_if(edges_.begin(), edges_.end(), 
		 [&from, &to](const NavGraphEdge &edge) {
		   return edge.from() == from && edge.to() == to;
		 });
  return (e != edges_.end());
}

/** Add a node.
 * @param node node to add
 * @throw Exception thrown if node with the same name as @p node already exists
 */
void
NavGraph::add_node(const NavGraphNode &node)
{
  if (node_exists(node)) {
    throw Exception("Node with name %s already exists", node.name().c_str());
  } else {
    nodes_.push_back(node);
    reachability_calced_ = false;
    notify_of_change();
  }
}

///@cond INTERNAL
template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(T x, T y, int ulp)
{
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return (std::abs(x-y) < std::numeric_limits<T>::epsilon() * std::abs(x+y) * ulp)
    // unless the result is subnormal
    || std::abs(x-y) < std::numeric_limits<T>::min();
}
///@endcond INTERNAL

/** Add a node and connect it to the graph.
 * The node is added similar to add_node(). Then, an edge is added connecting the
 * node to the graph. There are two principal methods available:
 * CLOSEST_NODE: simply connect to an existing node closest to the given node
 * CLOSEST_EDGE: connect node to the edge in which segment it lies,
 *   i.e. search for an edge where we can find a perpendicular line
 *   going through the given node and any point on the edge's line
 *   segment. If no such segment is found, the node cannot be added.
 * CLOSEST_EDGE_OR_NODE: first try CLOSEST_EDGE method, if that fails
 * use CLOSEST_NODE.
 * @param node node to add
 * @param conn_mode connection mode to use
 */
void
NavGraph::add_node_and_connect(const NavGraphNode &node, ConnectionMode conn_mode)
{
  add_node(node);
  switch (conn_mode) {
  case CLOSEST_NODE:
    connect_node_to_closest_node(node);
    break;

  case CLOSEST_EDGE:
    connect_node_to_closest_edge(node);
    break;

  case CLOSEST_EDGE_OR_NODE:
    try {
      connect_node_to_closest_edge(node);
    } catch (Exception &e) {
      connect_node_to_closest_node(node);
    }
    break;
  }
}

/** Connect node to closest node.
 * @param n node to connect to closest node
 */
void
NavGraph::connect_node_to_closest_node(const NavGraphNode &n)
{
  NavGraphNode closest = closest_node_to(n.name());
  add_edge(NavGraphEdge(n.name(), closest.name()));
}


/** Connect node to closest edge
 * @param n node to connect to closest node
 */
void
NavGraph::connect_node_to_closest_edge(const NavGraphNode &n)
{
  NavGraphEdge closest = closest_edge(n.x(), n.y());
  cart_coord_2d_t p = closest.closest_point_on_edge(n.x(), n.y());

  NavGraphNode closest_conn = closest_node(p.x, p.y);
  NavGraphNode cn;
  if (almost_equal(closest_conn.distance(p.x, p.y), 0.f, 2)) {
    cn = closest_conn;
  } else {
    cn = NavGraphNode(NavGraph::format_name("C_%s", n.name().c_str()), p.x, p.y);
  }

  if (closest.from() == cn.name() || closest.to() == cn.name()) {
    // we actually want to connect to one of the end nodes of the edge,
    // simply add the new edge and we are done
    NavGraphEdge new_edge(cn.name(), n.name());
    new_edge.set_property("generated", true);
    add_edge(new_edge);
  } else {
    // we are inserting a new point into the edge
    remove_edge(closest);
    NavGraphEdge new_edge_1(closest.from(), cn.name());
    NavGraphEdge new_edge_2(closest.to(), cn.name());
    NavGraphEdge new_edge_3(cn.name(), n.name());
    new_edge_1.set_property("generated", true);
    new_edge_2.set_property("generated", true);
    new_edge_3.set_property("generated", true);

    if (! node_exists(cn))  add_node(cn);
    add_edge(new_edge_1);
    add_edge(new_edge_2);
    add_edge(new_edge_3);
  }
}


/** Add an edge
 * @param edge edge to add
 */
void
NavGraph::add_edge(const NavGraphEdge &edge)
{
  if (edge_exists(edge)) {
    throw Exception("Edge from %s to %s already exists",
		    edge.from().c_str(), edge.to().c_str());
  } else {
    edges_.push_back(edge);
    edges_.back().set_nodes(node(edge.from()), node(edge.to()));
    reachability_calced_ = false;
    notify_of_change();
  }
}


/** Remove a node.
 * @param node node to remove
 */
void
NavGraph::remove_node(const NavGraphNode &node)
{
  std::remove(nodes_.begin(), nodes_.end(), node);
  edges_.erase(
    std::remove_if(edges_.begin(), edges_.end(),
		   [&node](const NavGraphEdge &edge)->bool {
		     return edge.from() == node.name() || edge.to() == node.name();
		   }), edges_.end());
  reachability_calced_ = false;
  notify_of_change();
}

/** Remove a node.
 * @param node_name name of node to remove
 */
void
NavGraph::remove_node(const std::string &node_name)
{
  nodes_.erase(
    std::remove_if(nodes_.begin(), nodes_.end(),
		   [&node_name](const NavGraphNode &node)->bool {
		     return node.name() == node_name;
		   }), nodes_.end());
  edges_.erase(
    std::remove_if(edges_.begin(), edges_.end(),
		   [&node_name](const NavGraphEdge &edge)->bool {
		     return edge.from() == node_name || edge.to() == node_name;
		   }), edges_.end());
  reachability_calced_ = false;
  notify_of_change();
}

/** Remove an edge
 * @param edge edge to remove
 */
void
NavGraph::remove_edge(const NavGraphEdge &edge)
{
  edges_.erase(
    std::remove_if(edges_.begin(), edges_.end(),
		   [&edge](const NavGraphEdge &e)->bool {
		     return (edge.from() == e.from() && edge.to() == e.to()) ||
		       (! e.is_directed() && (edge.from() == e.to() && edge.to() == e.from()));
		   }), edges_.end());
  reachability_calced_ = false;
  notify_of_change();
}

/** Remove an edge
 * @param from originating node name
 * @param to target node name
 */
void
NavGraph::remove_edge(const std::string &from, const std::string &to)
{
  edges_.erase(
    std::remove_if(edges_.begin(), edges_.end(),
		   [&from, &to](const NavGraphEdge &edge)->bool {
		     return (edge.from() == from && edge.to() == to) ||
		       (! edge.is_directed() && (edge.to() == from && edge.from() == to));
		   }), edges_.end());
  reachability_calced_ = false;
  notify_of_change();
}


/** Update a given node.
 * Will search for a node with the same name as the given node and will then
 * call the assignment operator. This is intended to update properties of a node.
 * @param node node to update
 */
void
NavGraph::update_node(const NavGraphNode &node)
{
  std::vector<NavGraphNode>::iterator n =
    std::find(nodes_.begin(), nodes_.end(), node);
  if (n != nodes_.end()) {
    *n = node;
  } else {
    throw Exception("No node with name %s known", node.name().c_str());
  }
}

/** Update a given edge.
 * Will search for an edge with the same originating and target node as the
 * given edge and will then call the assignment operator. This is intended
 * to update properties of an edge.
 * @param edge edge to update
 */
void
NavGraph::update_edge(const NavGraphEdge &edge)
{
  std::vector<NavGraphEdge>::iterator e =
    std::find(edges_.begin(), edges_.end(), edge);
  if (e != edges_.end()) {
    *e = edge;
  } else {
    throw Exception("No edge from %s to %s is known",
		    edge.from().c_str(), edge.to().c_str());
  }
}


/** Remove all nodes and edges from navgraph.
 * Use with caution, this means that no more searches etc. are possible.
 */
void
NavGraph::clear()
{
  nodes_.clear();
  edges_.clear();
  default_properties_.clear();
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
NavGraph::has_default_property(const std::string &property) const
{
  return default_properties_.find(property) != default_properties_.end();
}

/** Get specified default property as string.
 * @param prop property key
 * @return default property value as string
 */
std::string
NavGraph::default_property(const std::string &prop) const
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
NavGraph::default_property_as_float(const std::string &prop) const
{
  return StringConversions::to_float(default_property(prop));
}

/** Get property converted to int.
 * @param prop property key
 * @return property value
 */
int
NavGraph::default_property_as_int(const std::string &prop) const
{
  return StringConversions::to_int(default_property(prop));
}

/** Get property converted to bol.
 * @param prop property key
 * @return property value
 */
bool
NavGraph::default_property_as_bool(const std::string &prop) const
{
  return StringConversions::to_bool(default_property(prop));
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraph::set_default_property(const std::string &property, const std::string &value)
{
  default_properties_[property] = value;
}

/** Set default properties.
 * This overwrites all existing properties.
 * @param properties map of property name to value as string
 */
void
NavGraph::set_default_properties(const std::map<std::string, std::string> &properties)
{
  default_properties_ = properties;
}


/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraph::set_default_property(const std::string &property, float value)
{
  default_properties_[property] = StringConversions::to_string(value);
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraph::set_default_property(const std::string &property, int value)
{
  default_properties_[property] = StringConversions::to_string(value);
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraph::set_default_property(const std::string &property, bool value)
{
  default_properties_[property] = value ? "true" : "false";
}



/** Get nodes reachable from specified nodes.
 * @param node_name name of the node to get reachable nodes for
 * @return vector of names of nodes reachable from the specified node
 */
std::vector<std::string>
NavGraph::reachable_nodes(const std::string &node_name) const
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
  if (! reachability_calced_)  calc_reachability();

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
 * @param allow_multi_graph if true, allows multiple disconnected graph segments.
 */
void
NavGraph::calc_reachability(bool allow_multi_graph)
{
  if (nodes_.empty())  return;

  assert_valid_edges();

  std::vector<NavGraphNode>::iterator i;
  for (i = nodes_.begin(); i != nodes_.end(); ++i) {
    i->set_reachable_nodes(reachable_nodes(i->name()));
  }

  std::vector<NavGraphEdge>::iterator e;
  for (e = edges_.begin(); e != edges_.end(); ++e) {
    e->set_nodes(node(e->from()), node(e->to()));
  }

  if (! allow_multi_graph)  assert_connected();
  reachability_calced_ = true;
}

/** Create node name from a format string.
 * @param format format for the name according to sprintf arguments
 * @param ... parameters according to format
 * @return generated name
 */
std::string
NavGraph::format_name(const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  char *tmp;
  std::string rv;
  if (vasprintf(&tmp, format, arg) != -1) {
    rv = tmp;
    free(tmp);
  }
  va_end(arg);
  return rv;
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
