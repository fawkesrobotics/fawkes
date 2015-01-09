
/***************************************************************************
 *  navgraph.h - Topological graph
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

#include <navgraph/navgraph_node.h>
#include <navgraph/navgraph_edge.h>
#include <core/utils/lockptr.h>

#include <vector>
#include <list>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphConstraintRepo;

class NavGraph
{
 public:
  NavGraph(std::string graph_name);
  virtual ~NavGraph();
  
  std::string                              name() const;
  const std::vector<NavGraphNode> &        nodes() const;
  const std::vector<NavGraphEdge> &        edges() const;
  fawkes::LockPtr<NavGraphConstraintRepo>  constraint_repo() const;

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

  NavGraphNode node(std::string name) const;
  bool node_exists(std::string name) const;

  NavGraphNode closest_node(float pos_x, float pos_y,
                                  std::string property = "");

  NavGraphNode closest_node_to(std::string node_name,
				     std::string property = "");

  NavGraphNode closest_node(float pos_x, float pos_y, bool consider_unconnected,
                                  std::string property = "");
  
  NavGraphNode closest_node_to(std::string node_name, bool consider_unconnected,
				     std::string property = "");

  NavGraphNode closest_node_with_unconnected(float pos_x, float pos_y,
						   std::string property = "");

  NavGraphNode closest_node_to_with_unconnected(std::string node_name,
						      std::string property = "");

  std::vector<NavGraphNode> search_nodes(std::string property);

  std::vector<std::string>  reachable_nodes(std::string node_name) const;

  void add_node(NavGraphNode node);
  void add_edge(NavGraphEdge edge);

  void calc_reachability();

  NavGraph & operator=(const NavGraph &g);

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
  std::string                             graph_name_;
  std::vector<NavGraphNode>               nodes_;
  std::vector<NavGraphEdge>               edges_;
  fawkes::LockPtr<NavGraphConstraintRepo> constraint_repo_;
  std::list<ChangeListener *>             change_listeners_;
  std::map<std::string, std::string>      default_properties_;

};


} // end of namespace fawkes

#endif
