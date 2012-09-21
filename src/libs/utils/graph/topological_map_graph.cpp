
/***************************************************************************
 *  topological_map_graph.cpp - Topological graph
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

#include <utils/graph/topological_map_graph.h>

#include <algorithm>
#include <cmath>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

TopologicalMapGraph::TopologicalMapGraph(std::string graph_name)
{
  graph_name_ = graph_name;
}

TopologicalMapGraph::~TopologicalMapGraph()
{
}

  
std::string
TopologicalMapGraph::name() const
{
  return graph_name_;
}


const std::vector<TopologicalMapNode> &
TopologicalMapGraph::nodes() const
{
  return nodes_;
}


const std::vector<TopologicalMapEdge> &
TopologicalMapGraph::edges() const
{
  return edges_;
}


TopologicalMapNode
TopologicalMapGraph::node(std::string name) const
{
  std::vector<TopologicalMapNode>::const_iterator i;
  for (i = nodes_.begin(); i != nodes_.end(); ++i) {
    if (i->name() == name)  return *i;
  }
  return TopologicalMapNode();
}


TopologicalMapNode
TopologicalMapGraph::root_node() const
{
  return root_node_;
}


TopologicalMapNode
TopologicalMapGraph::closest_node(float pos_x, float pos_y,
                                  std::string property)
{
  std::vector<TopologicalMapNode> nodes = search_nodes(property);

  float min_dist = HUGE;

  std::vector<TopologicalMapNode>::iterator i;
  std::vector<TopologicalMapNode>::iterator elem = nodes.begin();
  for (i = nodes.begin(); i != nodes.end(); ++i) {
    float dx   = i->x() - pos_x;
    float dy   = i->y() - pos_y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (sqrtf(dx * dx + dy * dy) < min_dist) {
      min_dist = dist;
      elem = i;
    }
  }

  if (elem == nodes.end()) {
    return TopologicalMapNode();
  } else {
    return *elem;
  }
}


bool
TopologicalMapGraph::node_exists(std::string name) const
{
  std::vector<TopologicalMapNode>::const_iterator i;
  for (i = nodes_.begin(); i != nodes_.end(); ++i) {
    if (i->name() == name)  return true;
  }
  return false;
}

std::vector<TopologicalMapNode>
TopologicalMapGraph::search_nodes(std::string property)
{
  if (property == "") {
    return nodes();
  } else {
    std::vector<TopologicalMapNode> rv;

    std::vector<TopologicalMapNode>::iterator i;
    for (i = nodes_.begin(); i != nodes_.end(); ++i) {
      if ( i->has_property(property) )  rv.push_back(*i);
    }

    return rv;
  }
}

void
TopologicalMapGraph::set_root(std::string node_id)
{
  root_node_ = node(node_id);
}


void
TopologicalMapGraph::add_node(TopologicalMapNode node)
{
  nodes_.push_back(node);
}

void
TopologicalMapGraph::add_edge(TopologicalMapEdge edge)
{
  edges_.push_back(edge);
}


std::vector<std::string>
TopologicalMapGraph::reachable_nodes(std::string node_name) const
{
  std::vector<std::string> rv;

  TopologicalMapNode n = node(node_name);
  if (! n.is_valid())  return rv;

  std::vector<TopologicalMapEdge>::const_iterator i;
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


void
TopologicalMapGraph::calc_reachability()
{
  std::vector<TopologicalMapNode>::iterator i;
  for (i = nodes_.begin(); i != nodes_.end(); ++i) {
    i->set_reachable_nodes(reachable_nodes(i->name()));
  }
}


} // end of namespace fawkes
