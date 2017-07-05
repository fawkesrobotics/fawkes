
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

  extern const char *PROP_ORIENTATION;
}

class NavGraphConstraintRepo;

class NavGraph
{
 public:
  /** Connect mode enum for connect_node_* methods. */
  typedef enum {
    CLOSEST_NODE,		///< Connect to closest node
    CLOSEST_EDGE,		///< Connect to closest edge
    CLOSEST_EDGE_OR_NODE	///< try to connect to closest edge,
				///< if that fails, connect to closest node
  } ConnectionMode;

  /** Mode to use to add edges. */
  typedef enum {
    EDGE_FORCE,		///< add nodes no matter what (be careful)
    EDGE_NO_INTERSECTION,	///< Only add edge if it does not intersect
				///< with any existing edge
    EDGE_SPLIT_INTERSECTION	///< Add the edge, but if it intersects with
				///< an existing edges add new points at the
				///< intersection points for both, the conflicting
				///< edges and the new edge
  } EdgeMode;

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

  void apply_default_properties(NavGraphNode &node);

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

  NavGraphEdge edge(const std::string &from, const std::string &to) const;
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
  void add_node_and_connect(const NavGraphNode &node, ConnectionMode conn_mode);
  void connect_node_to_closest_node(const NavGraphNode &n);
  void connect_node_to_closest_edge(const NavGraphNode &n);
  void add_edge(const NavGraphEdge &edge, EdgeMode mode = EDGE_NO_INTERSECTION,
		bool allow_existing = false);
  void remove_node(const NavGraphNode &node);
  void remove_node(const std::string &node_name);
  void remove_orphan_nodes();
  void remove_edge(const NavGraphEdge &edge);
  void remove_edge(const std::string &from, const std::string &to);
  void clear();

  void update_node(const NavGraphNode &node);
  void update_edge(const NavGraphEdge &edge);

  bool node_exists(const NavGraphNode &node) const;
  bool node_exists(const std::string &name) const;
  bool edge_exists(const NavGraphEdge &edge) const;
  bool edge_exists(const std::string &from, const std::string &to) const;

  void calc_reachability(bool allow_multi_graph = false);

  NavGraph & operator=(const NavGraph &g);

  void set_notifications_enabled(bool enabled);
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
  std::string gen_unique_name(const char *prefix = "U-");

 private:
  void assert_valid_edges();
  void assert_connected();
  void edge_add_no_intersection(const NavGraphEdge &edge);
  void edge_add_split_intersection(const NavGraphEdge &edge);

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


  bool                                    notifications_enabled_;
};


} // end of namespace fawkes

#endif
