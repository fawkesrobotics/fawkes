
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

#ifndef __LIBS_NAVGRAPH_NAVGRAPH_H_
#define __LIBS_NAVGRAPH_NAVGRAPH_H_

#include <navgraph/navgraph_node.h>
#include <navgraph/navgraph_edge.h>
#include <navgraph/navgraph_path.h>
#include <core/utils/lockptr.h>

#include <vector>
#include <list>
#include <string>
#include <functional>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

namespace navgraph {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif
  typedef
  std::function<float (const fawkes::NavGraphNode &, const fawkes::NavGraphNode &)>
      EstimateFunction;
  typedef
  std::function<float (const fawkes::NavGraphNode &, const fawkes::NavGraphNode &)>
      CostFunction;
}

class NavGraphConstraintRepo;

class NavGraph
{
 public:
  NavGraph(const std::string &graph_name);
  virtual ~NavGraph();
  
  std::string                              name() const;
  const std::vector<NavGraphNode> &        nodes() const;
  const std::vector<NavGraphEdge> &        edges() const;
  fawkes::LockPtr<NavGraphConstraintRepo>  constraint_repo() const;

  const std::map<std::string, std::string> &  default_properties() const;
  bool has_default_property(const std::string &property) const;

  std::string default_property(const std::string &prop) const;
  float default_property_as_float(const std::string &prop) const;
  int   default_property_as_int(const std::string &prop) const;
  bool  default_property_as_bool(const std::string &prop) const;

  void set_default_property(const std::string &property, const std::string &value);
  void set_default_property(const std::string &property, float value);
  void set_default_property(const std::string &property, int value);
  void set_default_property(const std::string &property, bool value);
  void set_default_properties(const std::map<std::string, std::string> &properties);

  NavGraphNode node(const std::string &name) const;

  NavGraphNode closest_node(float pos_x, float pos_y,
			    const std::string &property = "") const;

  NavGraphNode closest_node_to(const std::string &node_name,
			       const std::string &property = "") const;

  NavGraphNode closest_node(float pos_x, float pos_y, bool consider_unconnected,
			    const std::string &property = "") const;
  
  NavGraphNode closest_node_to(const std::string &node_name, bool consider_unconnected,
			       const std::string &property = "") const;

  NavGraphNode closest_node_with_unconnected(float pos_x, float pos_y,
					     const std::string &property = "") const;

  NavGraphNode closest_node_to_with_unconnected(const std::string &node_name,
						const std::string &property = "") const;

  NavGraphEdge closest_edge(float pos_x, float pos_y) const;

  std::vector<NavGraphNode> search_nodes(const std::string &property) const;

  std::vector<std::string>  reachable_nodes(const std::string &node_name) const;

  fawkes::NavGraphPath search_path(const std::string &from, const std::string &to,
				   bool use_constraints = true, bool compute_constraints = true);

  fawkes::NavGraphPath search_path(const std::string &from, const std::string &to,
				   navgraph::EstimateFunction estimate_func,
				   navgraph::CostFunction cost_func,
				   bool use_constraints = true, bool compute_constraints = true);

  fawkes::NavGraphPath search_path(const NavGraphNode &from,
				   const NavGraphNode &to,
				   bool use_constraints = true, bool compute_constraints = true);

  fawkes::NavGraphPath search_path(const NavGraphNode &from,
				   const NavGraphNode &to,
				   navgraph::EstimateFunction estimate_func,
				   navgraph::CostFunction cost_func,
				   bool use_constraints = true, bool compute_constraints = true);

  void add_node(const NavGraphNode &node);
  void add_edge(const NavGraphEdge &edge);
  void remove_node(const NavGraphNode &node);
  void remove_edge(const NavGraphEdge &edge);
  void clear();

  bool node_exists(const NavGraphNode &node) const;
  bool node_exists(const std::string &name) const;
  bool edge_exists(const NavGraphEdge &edge) const;
  bool edge_exists(const std::string &from, const std::string &to) const;

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


  /** Check if the default euclidean distance search is used.
   * @return true if the default cost and cost estimation functions
   * are used, false of custom ones have been set.
   */
  bool uses_default_search() const
  { return search_default_funcs_; }

  void set_search_funcs(navgraph::EstimateFunction estimate_func,
			navgraph::CostFunction     cost_func);

  void unset_search_funcs();

  float cost(const NavGraphNode &from, const NavGraphNode &to) const;

  static std::string format_name(const char *format, ...);

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

  bool                                    search_default_funcs_;
  navgraph::EstimateFunction              search_estimate_func_;
  navgraph::CostFunction                  search_cost_func_;

  bool                                    reachability_calced_;
};


} // end of namespace fawkes

#endif
