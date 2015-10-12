
/***************************************************************************
 *  navgraph_path.cpp - Topological graph - path
 *
 *  Created: Mon Jan 12 10:57:24 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#include <navgraph/navgraph_path.h>
#include <navgraph/navgraph.h>
#include <core/exceptions/software.h>
#include <utils/misc/string_split.h>

#include <cmath>
#include <algorithm>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphPath <navgraph/navgraph_path.h>
 * Class representing a path for a NavGraph.
 * A path is a consecutive sequence of nodes, where each node is either
 * the first node in the path or reachable by its predecessor.
 * A path may or may not specify a total cost. If the value is unavailable
 * it shall be less than zero. If positive, the cost is valid. The unit
 * of the cost is not specified but depends on the search metrics used
 * to determine the path. Make sure to only compare paths that have been
 * created with the same metrics.
 * @author Tim Niemueller
 */

/** Default constructor.
 * Creates an invalid path.
 */
NavGraphPath::NavGraphPath()
  : graph_(NULL)
{
  cost_ = -1;
}


/** Constructor.
 * @param graph navgraph this path is based on
 * @param nodes nodes that the path should follow. The nodes must build
 * a sequence where each node is directly reachable from its predecessor.
 * This is not verified internally.
 * @param cost cost of the path, set to a value less than zero if unknown
 */
NavGraphPath::NavGraphPath(const NavGraph *graph,
			   std::vector<NavGraphNode> &nodes, float cost)
  : graph_(graph), nodes_(nodes), cost_(cost)
{
}

/** Check if this path is cheaper than the other path.
 * If both paths have negative costs (the cost is unknown), then
 * they are considered to be equal. Only if both cost values are
 * positive are they compared.
 * @param p path to compare to
 * @return true if this path is cheaper in terms of cost than the
 * other path, false if both costs are negative or the other path
 * is cheaper.
 */
bool
NavGraphPath::operator<(const NavGraphPath &p) const
{
  if (cost_ < 0 && p.cost_ < 0)  return false;
  return cost_ < p.cost_;
}


/** Check if two paths are the same.
 * Two paths are the same iff they contain the same nodes in the
 * exact same order and if they have the same cost (within a small
 * epsilon of 0.00001 and only if both costs are positive). Costs
 * are ignored should any of the two cost values be less than zero
 * (unknown).
 * @param p path to compare to
 * @return true if the paths are the same by the definition above,
 * false otherwise.
 */
bool
NavGraphPath::operator==(const NavGraphPath &p) const
{
  if (nodes_.size() != p.nodes_.size())  return false;

  for (size_t i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i] != p.nodes_[i])  return false;
  }

  if (cost_ >= 0 && p.cost_ >= 0 && fabs(cost_ - p.cost_) <= 0.00001)  return false;

  return true;
}


/** Add a node to the path.
 * The node must be reachable directly from the last node in the
 * path (not verified internally) or the first node.
 * @param node node to add to the path
 * @param cost_from_end cost to the node from the current end of
 * the path. It is added to the current total cost. The value is
 * ignored if it is less than zero.
 */
void
NavGraphPath::add_node(const NavGraphNode &node, float cost_from_end)
{
  nodes_.push_back(node);
  if (cost_from_end > 0) {
    cost_ += cost_from_end;
  }
}


/** Set nodes erasing the current path.
 * @param nodes nodes that the path should follow. The nodes must build
 * a sequence where each node is directly reachable from its predecessor.
 * This is not verified internally. This also invalidates any running
 * traversal.
 * @param cost cost of the path, set to a value less than zero if unknown
 */
void
NavGraphPath::set_nodes(const std::vector<NavGraphNode> &nodes, float cost)
{
  nodes_        = nodes;
  cost_         = cost;
}


/** Check if path is empty.
 * @return true if path is empty, i.e. it has no nodes at all,
 * false otherwise.
 */
bool
NavGraphPath::empty() const
{
  return nodes_.empty();
}


/** Get size of path.
 * @return number of nodes in path
 */
size_t
NavGraphPath::size() const
{
  return nodes_.size();
}


/** Clear all nodes on this path.
 * This sets the length of the path to zero and cost to unknown.
 */
void
NavGraphPath::clear()
{
  nodes_.clear();
  cost_ = -1;
}


/** Check if the path contains a given node.
 * @param node node to check for in current path
 * @return true if the node is contained in the current path, false otherwise
 */
bool
NavGraphPath::contains(const NavGraphNode &node) const
{
  return (std::find(nodes_.begin(), nodes_.end(), node) != nodes_.end());
}


/** Get goal of path.
 * @return goal of this path, i.e. the last node in the sequence of nodes.
 * @throw Exeption if there are no nodes in this path
 */
const NavGraphNode &
NavGraphPath::goal() const
{
  if (nodes_.empty()) {
    throw Exception("No nodes in plan, cannot retrieve goal");
  }

  return nodes_[nodes_.size() - 1];
}


/** Get graph this path is based on.
 * @return const reference to graph this path is based on
 */
const NavGraph &
NavGraphPath::graph() const
{
  return *graph_;
}


/** Get a new path traversal handle.
 * @return new path traversal handle
 */
NavGraphPath::Traversal
NavGraphPath::traversal() const
{
  return Traversal(this);
}

/** @class NavGraphPath::Traversal <navgraph/navgraph_path.h>
 * Sub-class representing a navgraph path traversal.
 * A traversal is a step-by-step run through the node sequence (in order).
 * There maybe any number of traversal open for a given path. But they
 * become invalid should new nodes be set on the path.
 * After creating a new traversal, you always need to call next for
 * each new node including the first one. Code could look like this.
 * @code
 * NavGraphPath path = navgraph->search_path("from-here", "to-there");
 * NavGraphPath::Traversal traversal = path.traversal();
 *
 * while (traversal.next()) {
 *   const NavGraphNode &current = traversal.current();
 *   // operate on node
 *   if (traversal.last()) {
 *     // current is the last node on the path and traversal
 *     // will end after this iteration.
 *   }
 * }
 * @endcode
 * @author Tim Niemueller
 */


/** Constructor. */
NavGraphPath::Traversal::Traversal()
  : path_(NULL), current_(-1)
{
}


/** Constructor.
 * @param path parent path to traverse
 */
NavGraphPath::Traversal::Traversal(const NavGraphPath *path)
  : path_(path), current_(-1)
{
}


/** Invalidate this traversal.
 * This will reset the parent path and the current node.
 * This traversal can now longer be used afterwards other
 * than assigning a new traversal.
 */
void
NavGraphPath::Traversal::invalidate()
{
  current_ = -1;
  path_ = NULL;
}

void
NavGraphPath::Traversal::assert_initialized() const
{
  if (! path_) throw NullPointerException("Traversal has not been properly initialized");
}

/** Get current node in path.
 * @return current node in traversal
 * @throw Exception if no traversal is active, i.e. next() has not been called
 * after a traversal reset or if the path has already been traversed completley.
 */
const NavGraphNode &
NavGraphPath::Traversal::current() const
{
  assert_initialized();
  if (current_ >= 0 && current_ < (ssize_t)path_->nodes_.size()) {
    return path_->nodes_[current_];
  } else {
    throw OutOfBoundsException("No more nodes in path to query.");
  }
}


/** Peek on the next node.
 * Get the node following the current node without advancing
 * the current index (the current node remains the same).
 * @return node following the current node
 * @throw OutOfBoundsException if the traversal has not been
 * started with an initial call to next() or if the traversal
 * has already ended or is currently at the last node.
 */
const NavGraphNode &
NavGraphPath::Traversal::peek_next() const
{
  assert_initialized();
  if (current_ >= 0 && current_ < (ssize_t)path_->nodes_.size() - 1) {
    return path_->nodes_[current_ + 1];
  } else {
    throw OutOfBoundsException("Next node not available, cannot peek");
  }
}


/** Check if traversal is currently runnung.
 * @return true if current() will return a valid result
 */
bool
NavGraphPath::Traversal::running() const
{
  return (current_ >= 0 && current_ < (ssize_t)path_->nodes_.size());
}

/** Get index of current node in path.
 * @return index of current node in traversal
 * @throw Exception if no traversal is active, i.e. next() has not been called
 * after a traversal reset or if the path has already been traversed completley.
 */
size_t
NavGraphPath::Traversal::current_index() const
{
  assert_initialized();
  if (current_ >= 0 && current_ < (ssize_t)path_->nodes_.size()) {
    return current_;
  } else {
    throw OutOfBoundsException("No more nodes in path to query.");
  }
}

/** Move on traversal to next node.
 * @return bool, if there was another node to traverse, false otherwise
 */
bool
NavGraphPath::Traversal::next()
{
  assert_initialized();
  if (current_ < (ssize_t)path_->nodes_.size())  current_ += 1;

  return (current_ < (ssize_t)path_->nodes_.size());
}


/** Check if the current node is the last node in the path.
 * @return true if the current node is the last node in the path,
 * false otherwise
 */
bool
NavGraphPath::Traversal::last() const
{
  assert_initialized();
  return (current_ >= 0 && (size_t)current_ == (path_->nodes_.size() - 1));
}


/** Get the number of remaining nodes in path traversal.
 * The number of remaining nodes is the number of nodes
 * including the current node up until the last node in the path.
 * @return number of remaining nodes in path traversal
 */
size_t
NavGraphPath::Traversal::remaining() const
{
  assert_initialized();
  if (current_ < 0)  return path_->nodes_.size();
  return path_->nodes_.size() - (size_t)current_;
}


/** Get the remaining cost to the goal.
 * This sums the costs from the current to the goal node using
 * the default registered cost function of the parent navgraph.
 * @return cost from current to goal node
 */
float
NavGraphPath::Traversal::remaining_cost() const
{
  assert_initialized();
  if (! path_->graph_) {
    throw NullPointerException("Parent graph has not been set");
  }

  if (current_ < 0)  return path_->cost();

  float cost = 0.;
  for (ssize_t i = current_; i < (ssize_t)path_->nodes_.size() - 1; ++i) {
    cost += path_->graph_->cost(path_->nodes_[i], path_->nodes_[i+1]);
  }

  return cost;
}


/** Reset an ongoing traversal.
 * A new traversal afterwards will start the traversal from the beginning.
 */
void
NavGraphPath::Traversal::reset()
{
  current_ = -1;
}


/** Set the current node.
 * @param new_current new current node
 * @throw OutOfBoundsException thrown if new current node is beyond
 * number of nodes in path.
 */
void
NavGraphPath::Traversal::set_current(size_t new_current)
{
  assert_initialized();
  if (new_current >= path_->nodes_.size()) {
    throw OutOfBoundsException("Invalid new index, is beyond path length");
  }
  current_ = new_current;
}


/** Get string representation of path.
 * @param delim custom delimiter
 * @return all nodes of the path in one string
 */
std::string
NavGraphPath::get_path_as_string(const char delim) const
{
	return str_join(get_node_names(), delim);
}

/** Get names of nodes in path.
 * @return vector of strings for all nodes
 */
std::vector<std::string>
NavGraphPath::get_node_names() const
{
	std::vector<std::string> nodes(nodes_.size());
	for (size_t i = 0; i < nodes_.size(); ++i){
		nodes[i] = nodes_[i].name();
	}
	return nodes;
}

} // end of namespace fawkes
