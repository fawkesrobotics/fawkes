
/***************************************************************************
 *  topological_map_graph.h - Topological graph
 *
 *  Created: Fri Sep 21 15:48:00 2012
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

#ifndef __UTILS_GRAPH_TOPOLOGICAL_MAP_GRAPH_H_
#define __UTILS_GRAPH_TOPOLOGICAL_MAP_GRAPH_H_

#include <navgraph/topological_map_node.h>
#include <navgraph/topological_map_edge.h>

#include <vector>
#include <list>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class TopologicalMapGraph
{
 public:
  TopologicalMapGraph(std::string graph_name);
  virtual ~TopologicalMapGraph();
  
  std::string                              name() const;
  const std::vector<TopologicalMapNode> &  nodes() const;
  const std::vector<TopologicalMapEdge> &  edges() const;

  const std::map<std::string, std::string> &  default_properties() const;
  bool has_default_property(std::string property) const;

  std::string default_property(std::string prop) const;
  float default_property_as_float(std::string prop) const;
  int   default_property_as_int(std::string prop) const;
  bool  default_property_as_bool(std::string prop) const;

  void set_default_property(std::string property, std::string value);
  void set_default_property(std::string property, float value);
  void set_default_property(std::string property, int value);
  void set_default_property(std::string property, bool value);
  void set_default_properties(std::map<std::string, std::string> &properties);

  TopologicalMapNode node(std::string name) const;
  bool node_exists(std::string name) const;
  TopologicalMapNode root_node() const;

  TopologicalMapNode closest_node(float pos_x, float pos_y,
                                  std::string property = "");

  TopologicalMapNode closest_node_to(std::string node_name,
				     std::string property = "");

  TopologicalMapNode closest_node(float pos_x, float pos_y, bool consider_unconnected,
                                  std::string property = "");
  
  TopologicalMapNode closest_node_to(std::string node_name, bool consider_unconnected,
				     std::string property = "");

  TopologicalMapNode closest_node_with_unconnected(float pos_x, float pos_y,
						   std::string property = "");

  TopologicalMapNode closest_node_to_with_unconnected(std::string node_name,
						      std::string property = "");

  std::vector<TopologicalMapNode> search_nodes(std::string property);

  std::vector<std::string>  reachable_nodes(std::string node_name) const;

  void set_root(std::string node_id);
  void add_node(TopologicalMapNode node);
  void add_edge(TopologicalMapEdge edge);

  void calc_reachability();

  TopologicalMapGraph & operator=(const TopologicalMapGraph &g);

  void notify_of_change() throw();

  class ChangeListener {
   public:
    virtual ~ChangeListener();
    virtual void graph_changed() throw() = 0;
  };

  void add_change_listener(ChangeListener *listener);
  void remove_change_listener(ChangeListener *listener);

 private:
  void assert_unique_edges();
  void assert_valid_edges();
  void assert_unique_nodes();
  void assert_connected();

 private:
  TopologicalMapNode              root_node_;
  std::string                     graph_name_;
  std::vector<TopologicalMapNode> nodes_;
  std::vector<TopologicalMapEdge> edges_;
  std::list<ChangeListener *>     change_listeners_;
  std::map<std::string, std::string> default_properties_;

};


} // end of namespace fawkes

#endif
