/***************************************************************************
 *  navgraph_stconstr_thread.cpp - static constraints for navgraph
 *
 *  Created: Fri Jul 11 17:34:18 2014
 *  Copyright  2012-2014  Tim Niemueller [www.niemueller.de]
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

#include "navgraph_stconstr_thread.h"

#include <navgraph/constraints/static_list_node_constraint.h>
#include <navgraph/constraints/static_list_edge_constraint.h>
#include <navgraph/constraints/static_list_edge_cost_constraint.h>
#include <navgraph/constraints/polygon_node_constraint.h>
#include <navgraph/constraints/polygon_edge_constraint.h>
#include <utils/misc/string_split.h>

using namespace fawkes;

/** @class NavGraphStaticConstraintsThread "navgraph_stconstr_thread.h"
 * Thread to statically block certain nodes from config.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphStaticConstraintsThread::NavGraphStaticConstraintsThread()
  : Thread("NavGraphStaticConstraintsThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
NavGraphStaticConstraintsThread::~NavGraphStaticConstraintsThread()
{
}

void
NavGraphStaticConstraintsThread::init()
{
  std::vector<std::string> nodes =
    config->get_strings("/navgraph/static-constraints/nodes");

  std::vector<std::string> c_edges =
    config->get_strings("/navgraph/static-constraints/edges");

  std::vector<std::string> c_edge_costs =
    config->get_strings("/navgraph/static-constraints/edge-costs");

  std::vector<std::string> c_polygons =
    config->get_strings("/navgraph/static-constraints/polygons");

  std::vector<std::pair<std::string, std::string>> edges;
  for (std::string & ce : c_edges) {
    std::vector<std::string> node_names = str_split(ce, "--");
    if (node_names.size() == 2) {
      edges.push_back(std::make_pair(node_names[0], node_names[1]));
    }
  }

  std::vector<std::tuple<std::string, std::string, float>> edge_costs;
  for (const std::string & cec : c_edge_costs) {
    std::vector<std::string> nodes_cost = str_split(cec, ":");
    if (nodes_cost.size() != 2) {
      throw Exception("Invalid edge costs (colon): %s", cec.c_str());
    }
    std::vector<std::string> node_names = str_split(nodes_cost[0], "--");
    if (node_names.size() != 2) {
      throw Exception("Invalid edge costs (node names): %s", cec.c_str());
    }

    edge_costs.push_back(std::make_tuple(node_names[0], node_names[1],
					 StringConversions::to_float(nodes_cost[1])));
  }

  std::vector<NavGraphPolygonConstraint::Polygon> polygons;
  for (std::string & ce : c_polygons) {
    std::vector<std::string> points = str_split(ce);
    if (points.size() < 2) {
      throw Exception("Invalid polygon, must have at least two nodes");
    }
    NavGraphPolygonConstraint::Polygon polygon;
    for (const std::string &p : points) {
      std::vector<std::string> coord = str_split(p, ":");
      if (coord.size() != 2) {
	throw Exception("Polygon constraint with invalid point %s", p.c_str());
      }
      const NavGraphPolygonConstraint::Point
	polpoint(StringConversions::to_float(coord[0]),
		 StringConversions::to_float(coord[1]));
      polygon.push_back(polpoint);
    }
    if (polygon.front().x != polygon.back().x || polygon.front().y != polygon.back().y) {
      logger->log_info(name(), "Auto-circling constraint polygon %s",
		       ce.c_str());
      polygon.push_back(NavGraphPolygonConstraint::Point(polygon.front().x,
							 polygon.front().y));
    }
    polygons.push_back(polygon);
  }

  node_constraint_ = new NavGraphStaticListNodeConstraint("static-nodes");
  edge_constraint_ = new NavGraphStaticListEdgeConstraint("static-edges");
  edge_cost_constraint_ = new NavGraphStaticListEdgeCostConstraint("static-edge-cost");
  node_poly_constraint_ = new NavGraphPolygonNodeConstraint("static-node-polygon");
  edge_poly_constraint_ = new NavGraphPolygonEdgeConstraint("static-edge-polygon");

  const std::vector<NavGraphNode> &graph_nodes = navgraph->nodes();

  std::list<std::string> missing_nodes;
  for (std::string node_name : nodes) {
    bool found = false;
    for (const NavGraphNode &gnode : graph_nodes) {
      if (gnode.name() == node_name) {
	node_constraint_->add_node(gnode);
	found = true;
	break;
      }
    }

    if (!found) {
      missing_nodes.push_back(node_name);
    }
  }

  if (! missing_nodes.empty()) {
    std::list<std::string>::iterator n = missing_nodes.begin();
    std::string err_str = *n++;
    for (;n != missing_nodes.end(); ++n) {
      err_str += ", " + *n;
    }

    delete node_constraint_;
    delete edge_constraint_;
    delete edge_cost_constraint_;
    throw Exception("Some block nodes are not in graph: %s", err_str.c_str());
  }

  const std::vector<NavGraphEdge> &graph_edges = navgraph->edges();


  std::list<std::pair<std::string, std::string>> missing_edges;
  for (std::pair<std::string, std::string> edge : edges) {
    bool found = false;
    for (const NavGraphEdge &gedge : graph_edges) {
      if ((edge.first == gedge.from() && edge.second == gedge.to()) ||
	  (edge.first == gedge.to() && edge.second == gedge.from()))
      {
	edge_constraint_->add_edge(gedge);
	found = true;
	break;
      }
    }

    if (!found) {
      missing_edges.push_back(edge);
    }
  }

  if (! missing_edges.empty()) {
    std::list<std::pair<std::string, std::string>>::iterator n = missing_edges.begin();
    std::string err_str = n->first + "--" + n->second;
    for (++n ; n != missing_edges.end(); ++n) {
      err_str += ", " + n->first + "--" + n->second;
    }

    delete node_constraint_;
    delete edge_constraint_;
    delete edge_cost_constraint_;
    throw Exception("Some blocked edges are not in graph: %s", err_str.c_str());
  }

  missing_edges.clear();
  for (std::tuple<std::string, std::string, float> edge : edge_costs) {
    bool found = false;
    for (const NavGraphEdge &gedge : graph_edges) {
      if ((std::get<0>(edge) == gedge.from() && std::get<1>(edge) == gedge.to()) ||
	  (std::get<0>(edge) == gedge.to() && std::get<1>(edge) == gedge.from()))
      {
	edge_cost_constraint_->add_edge(gedge, std::get<2>(edge));
	found = true;
	break;
      }
    }

    if (!found) {
      missing_edges.push_back(std::make_pair(std::get<0>(edge), std::get<1>(edge)));
    }
  }

  if (! missing_edges.empty()) {
    std::list<std::pair<std::string, std::string>>::iterator n = missing_edges.begin();
    std::string err_str = n->first + "--" + n->second;
    for (++n ; n != missing_edges.end(); ++n) {
      err_str += ", " + n->first + "--" + n->second;
    }

    delete node_constraint_;
    delete edge_constraint_;
    delete edge_cost_constraint_;
    throw Exception("Some edges for cost factors are not in graph: %s", err_str.c_str());
  }


  for (const NavGraphPolygonConstraint::Polygon &p : polygons) {
    node_poly_constraint_->add_polygon(p);
    edge_poly_constraint_->add_polygon(p);
  }
  

  /*
  NavGraphPolygonNodeConstraint *pc = new NavGraphPolygonNodeConstraint("Poly");
  NavGraphPolygonNodeConstraint::Polygon p;
  p.push_back(NavGraphPolygonNodeConstraint::Point(0.0, 0.0));
  p.push_back(NavGraphPolygonNodeConstraint::Point(1.0, 0.0));
  p.push_back(NavGraphPolygonNodeConstraint::Point(1.0, 1.11));
  p.push_back(NavGraphPolygonNodeConstraint::Point(0.0, 1.11));
  p.push_back(NavGraphPolygonNodeConstraint::Point(0.0, 0.0));
  pc->add_polygon(p);

  NavGraphPolygonEdgeConstraint *pc = new NavGraphPolygonEdgeConstraint("Poly");
  NavGraphPolygonEdgeConstraint::Polygon p;
  p.push_back(NavGraphPolygonConstraint::Point(0.0, 0.0));
  p.push_back(NavGraphPolygonConstraint::Point(1.0, 0.0));
  p.push_back(NavGraphPolygonConstraint::Point(1.0, 1.11));
  p.push_back(NavGraphPolygonConstraint::Point(0.0, 1.11));
  p.push_back(NavGraphPolygonConstraint::Point(0.0, 0.0));
  pc->add_polygon(p);
  */

  navgraph->constraint_repo()->register_constraint(node_constraint_);
  navgraph->constraint_repo()->register_constraint(edge_constraint_);
  navgraph->constraint_repo()->register_constraint(edge_cost_constraint_);
  navgraph->constraint_repo()->register_constraint(node_poly_constraint_);
  navgraph->constraint_repo()->register_constraint(edge_poly_constraint_);
}

void
NavGraphStaticConstraintsThread::finalize()
{
  navgraph->constraint_repo()->unregister_constraint(node_constraint_->name());
  navgraph->constraint_repo()->unregister_constraint(edge_constraint_->name());
  navgraph->constraint_repo()->unregister_constraint(edge_cost_constraint_->name());
  delete node_constraint_;
  delete edge_constraint_;
  delete edge_cost_constraint_;
}

void
NavGraphStaticConstraintsThread::loop()
{
}
