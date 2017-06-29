
/***************************************************************************
 *  yaml_navgraph.cpp - Nav graph stored in a YAML file
 *
 *  Created: Fri Sep 21 18:37:16 2012
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

#include <navgraph/yaml_navgraph.h>
#include <navgraph/navgraph.h>
#include <core/exception.h>

#include <yaml-cpp/yaml.h>
#include <fstream>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Read topological map node from YAML iterator.
 * @param n iterator to node representing a topological map graph node
 * @param node node to fill
 */
static void
operator >> (const YAML::Node& n, NavGraphNode &node) {
#ifdef HAVE_YAMLCPP_0_5
  const std::string name = n["name"].as<std::string>();
#else
  std::string name;
  n["name"] >> name;
#endif

#ifdef HAVE_OLD_YAMLCPP
  if (n.GetType() != YAML::CT_MAP) {
#else
  if (n.Type() != YAML::NodeType::Map) {
#endif
    throw Exception("Node %s is not a map!?", name.c_str());
  }

  try {
    if (n["pos"].size() != 2) {
      throw Exception("Invalid position for node %s, "
                      "must be list of [x,y] coordinates", name.c_str());
    }
    float x, y;
#ifdef HAVE_YAMLCPP_0_5
    x = n["pos"][0].as<float>();
    y = n["pos"][1].as<float>();
#else
    n["pos"][0] >> x;
    n["pos"][1] >> y;
#endif

    node.set_x(x);
    node.set_y(y);
  } catch (YAML::Exception &e) {
    throw fawkes::Exception("Failed to parse node: %s", e.what());
  }

#ifdef HAVE_OLD_YAMLCPP
  if (n.GetTag() == "tag:fawkesrobotics.org,navgraph/unconnected") {
#else
  if (n.Tag() == "tag:fawkesrobotics.org,navgraph/unconnected") {
#endif
    node.set_unconnected(true);
  }

  bool has_properties = true;
  try {
#ifdef HAVE_YAMLCPP_0_5
    has_properties = n["properties"].IsDefined();
#else
    has_properties = (n.FindValue("properties") != NULL);
#endif
  } catch (YAML::Exception &e) {
    has_properties = false;
  }

  if (has_properties) {
    try {
      const YAML::Node &props = n["properties"];
      if (props.Type() != YAML::NodeType::Sequence) {
	throw Exception("Properties must be a list");
      }

      std::map<std::string, std::string> properties;

#ifdef HAVE_YAMLCPP_0_5
      YAML::const_iterator p;
#else
      YAML::Iterator p;
#endif
      for (p = props.begin(); p != props.end(); ++p) {
#ifdef HAVE_OLD_YAMLCPP
	if (p->GetType() == YAML::CT_SCALAR) {
#else
	if (p->Type() == YAML::NodeType::Scalar) {
#endif
#ifdef HAVE_YAMLCPP_0_5
	  std::string key = p->as<std::string>();
#else
	  std::string key;
	  *p >> key;
#endif
	  node.set_property(key, "true");
#ifdef HAVE_OLD_YAMLCPP
	} else if (p->GetType() == YAML::CT_MAP) {
#else
	} else if (p->Type() == YAML::NodeType::Map) {
#endif
#ifdef HAVE_YAMLCPP_0_5
	  for (YAML::const_iterator i = p->begin(); i != p->end(); ++i) {
	    std::string key   = i->first.as<std::string>();
	    std::string value = i->second.as<std::string>();
#else
	  for (YAML::Iterator i = p->begin(); i != p->end(); ++i) {
	    std::string key, value;
	    i.first() >> key;
	    i.second() >> value;
#endif
	    node.set_property(key, value);
	  }
	} else {
	  throw Exception("Invalid property for node '%s'", name.c_str());
	}
      }    
    } catch (YAML::Exception &e) {
      throw Exception("Failed to read propery of %s: %s",
		      name.c_str(), e.what());
    } // ignored
  }

  node.set_name(name);
}

/** Read topological map edge from YAML iterator.
 * @param n iterator to node representing a topological map graph edge
 * @param edge edge to fill
 */
static void
operator >> (const YAML::Node& n, NavGraphEdge &edge) {
#ifdef HAVE_OLD_YAMLCPP
  if (n.GetType() != YAML::CT_SEQUENCE || n.size() != 2) {
#else
  if (n.Type() != YAML::NodeType::Sequence || n.size() != 2) {
#endif
    throw Exception("Invalid edge");
  }
  std::string from, to;
#ifdef HAVE_YAMLCPP_0_5
  from = n[0].as<std::string>();
  to   = n[1].as<std::string>();
#else
  n[0] >> from;
  n[1] >> to;
#endif

  edge.set_from(from);
  edge.set_to(to);

#ifdef HAVE_OLD_YAMLCPP
  if (n.GetTag() == "tag:fawkesrobotics.org,navgraph/dir") {
#else
  if (n.Tag() == "tag:fawkesrobotics.org,navgraph/dir") {
#endif
    edge.set_directed(true);
  }

#ifdef HAVE_OLD_YAMLCPP
  if (n.GetTag() == "tag:fawkesrobotics.org,navgraph/allow-intersection") {
#else
  if (n.Tag() == "tag:fawkesrobotics.org,navgraph/allow-intersection") {
#endif
    edge.set_property("insert-mode", "force");
  }

#ifdef HAVE_OLD_YAMLCPP
  if (n.GetTag() == "tag:fawkesrobotics.org,navgraph/no-intersection") {
#else
  if (n.Tag() == "tag:fawkesrobotics.org,navgraph/no-intersection") {
#endif
    edge.set_property("insert-mode", "no-intersection");
  }

#ifdef HAVE_OLD_YAMLCPP
  if (n.GetTag() == "tag:fawkesrobotics.org,navgraph/split-intersection") {
#else
  if (n.Tag() == "tag:fawkesrobotics.org,navgraph/split-intersection") {
#endif
    edge.set_property("insert-mode", "split-intersection");
  }

}

/** Read default properties for graph from YAML node.
 * @param graph the graph to assign the properties to
 * @param doc the root document of the YAML graph definition
 */
void
read_default_properties(NavGraph *graph, YAML::Node &doc)
{
  bool has_properties = true;
  try {
#ifdef HAVE_YAMLCPP_0_5
    has_properties = doc["default-properties"].IsDefined();
#else
    has_properties = (doc.FindValue("default-properties") != NULL);
#endif
  } catch (YAML::Exception &e) {
    has_properties = false;
  }

  if (has_properties) {
    try {
      const YAML::Node &props = doc["default-properties"];
      if (props.Type() != YAML::NodeType::Sequence) {
	throw Exception("Default properties must be a list");
      }

      std::map<std::string, std::string> properties;

#ifdef HAVE_YAMLCPP_0_5
      YAML::const_iterator p;
#else
      YAML::Iterator p;
#endif
      for (p = props.begin(); p != props.end(); ++p) {
#ifdef HAVE_OLD_YAMLCPP
	if (p->GetType() == YAML::CT_SCALAR) {
#else
	if (p->Type() == YAML::NodeType::Scalar) {
#endif
#ifdef HAVE_YAMLCPP_0_5
	  std::string key = p->as<std::string>();
#else
	  std::string key;
	  *p >> key;
#endif
	  properties[key] = "true";
#ifdef HAVE_OLD_YAMLCPP
	} else if (p->GetType() == YAML::CT_MAP) {
#else
	} else if (p->Type() == YAML::NodeType::Map) {
#endif
#ifdef HAVE_YAMLCPP_0_5
	  for (YAML::const_iterator i = p->begin(); i != p->end(); ++i) {
	    std::string key   = i->first.as<std::string>();
	    std::string value = i->second.as<std::string>();
#else
	  for (YAML::Iterator i = p->begin(); i != p->end(); ++i) {
	    std::string key, value;
	    i.first() >> key;
	    i.second() >> value;
#endif
	    properties[key] = value;
	  }
	} else {
	  throw Exception("Invalid default property for graph %s", graph->name().c_str());
	}
      }    

      graph->set_default_properties(properties);
    } catch (YAML::Exception &e) {
      throw Exception("Failed to read default property of graph %s: %s",
	    graph->name().c_str(), e.what());
    }
  }
}

/** Load topological map graph stored in RCSoft format.
 * @param filename path to the file to read
 * @param allow_multi_graph if true, allows multiple disconnected graph segments
 * @return topological map graph read from file
 * @exception Exception thrown on any error to read the graph file
 */
NavGraph *
load_yaml_navgraph(std::string filename, bool allow_multi_graph)
{
  //try to fix use of relative paths
  if (filename[0] != '/') {
    filename = std::string(CONFDIR) + "/" + filename;
  }

  YAML::Node doc;
#ifdef HAVE_YAMLCPP_0_5
  if (! (doc = YAML::LoadFile(filename))) {
#else
  std::ifstream fin(filename.c_str());
  YAML::Parser parser(fin);
  if (! parser.GetNextDocument(doc)) {
#endif
    throw fawkes::Exception("Failed to read YAML file %s", filename.c_str());
  }

#ifdef HAVE_YAMLCPP_0_5
  std::string graph_name = doc["graph-name"].as<std::string>();
#else
  std::string graph_name;
  doc["graph-name"] >> graph_name;
#endif

  NavGraph *graph = new NavGraph(graph_name);

  read_default_properties(graph, doc);

  const YAML::Node &ynodes = doc["nodes"];
#ifdef HAVE_YAMLCPP_0_5
  for (YAML::const_iterator n = ynodes.begin(); n != ynodes.end(); ++n) {
#else
  for (YAML::Iterator n = ynodes.begin(); n != ynodes.end(); ++n) {
#endif
    NavGraphNode node;
    *n >> node;
    graph->add_node(node);
  }

  const YAML::Node &yedges = doc["connections"];
#ifdef HAVE_YAMLCPP_0_5
  for (YAML::const_iterator e = yedges.begin(); e != yedges.end(); ++e) {
#else
  for (YAML::Iterator e = yedges.begin(); e != yedges.end(); ++e) {
#endif
    NavGraphEdge edge;
    *e >> edge;
    if (edge.has_property("insert-mode")) {
      std::string mode = edge.property("insert-mode");
      if (mode == "force") {
        graph->add_edge(edge, NavGraph::EDGE_FORCE);
      } else if (mode == "no-intersection") {
        graph->add_edge(edge, NavGraph::EDGE_NO_INTERSECTION);
      } else if (mode == "split-intersection") {
        graph->add_edge(edge, NavGraph::EDGE_SPLIT_INTERSECTION);
      }
    } else {
      graph->add_edge(edge);
    }
  }

  graph->calc_reachability(allow_multi_graph);

  const std::vector<NavGraphNode> &nodes = graph->nodes();
  for (const NavGraphNode &n : nodes) {
    if (n.has_property("insert-mode")) {
      std::string ins_mode = n.property("insert-mode");
      if (ins_mode == "closest-node" || ins_mode == "CLOSEST_NODE") {
	graph->connect_node_to_closest_node(n);
      } else if (ins_mode == "closest-edge" || ins_mode == "CLOSEST_EDGE") {
	graph->connect_node_to_closest_edge(n);
      } else if (ins_mode == "closest-edge-or-node" || ins_mode == "CLOSEST_EDGE_OR_NODE") {
	try {
	  graph->connect_node_to_closest_edge(n);
	} catch (Exception &e) {
	  graph->connect_node_to_closest_node(n);
	}
      } else if (ins_mode == "unconnected" || ins_mode == "UNCONNECTED") {
	NavGraphNode updated_n(n);
	updated_n.set_unconnected(true);
	graph->update_node(updated_n);
      } // else NOT_CONNECTED, the default, do nothing here
    }
  }

  return graph;
}


/** Save navgraph to YAML file.
 * @param filename name of file to save to
 * @param graph graph to save to
 */
void
save_yaml_navgraph(std::string filename, NavGraph *graph)
{
  if (filename[0] != '/') {
    filename = std::string(CONFDIR) + "/" + filename;
  }

  YAML::Emitter out;
  out << YAML::TrueFalseBool
      << YAML::BeginMap
      << YAML::Key   << "graph-name"
      << YAML::Value << graph->name();

  const std::map<std::string, std::string> &def_props = graph->default_properties();
  if (! def_props.empty()) {
    out << YAML::Key   << "default-properties"
	<< YAML::Value << YAML::BeginSeq;
    for (auto &p : def_props) {
      out << YAML::BeginMap
	  << YAML::Key << p.first
	  << YAML::Value << p.second
	  << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }

  out << YAML::Key   << "nodes"
      << YAML::Value << YAML::BeginSeq;

  const std::vector<NavGraphNode> &nodes = graph->nodes();
  for (const NavGraphNode &node : nodes) {
    if (node.unconnected())  out << YAML::LocalTag("unconnected");
    out << YAML::BeginMap
	<< YAML::Key   << "name"
	<< YAML::Value << node.name()
	<< YAML::Key   << "pos"
	<< YAML::Value << YAML::Flow << YAML::BeginSeq << node.x() << node.y() << YAML::EndSeq;

    const std::map<std::string, std::string> &props = node.properties();
    if (! props.empty()) {
      out << YAML::Key   << "properties"
	  << YAML::Value << YAML::BeginSeq;
      for (auto &p : props) {
	out << YAML::BeginMap
	    << YAML::Key << p.first
	    << YAML::Value << p.second
	    << YAML::EndMap;
      }
      out << YAML::EndSeq;
    }

    out << YAML::EndMap;
  }
  out << YAML::EndSeq
      << YAML::Key   << "connections"
      << YAML::Value << YAML::BeginSeq;

  const std::vector<NavGraphEdge> &edges = graph->edges();
  for (const NavGraphEdge &edge : edges) {
    if (edge.is_directed())  out << YAML::LocalTag("dir");
    if (edge.has_property("insert-mode")) {
	    std::string insert_mode = edge.property("insert-mode");
	    if (insert_mode == "force") {
		    out << YAML::LocalTag("allow-intersection");
	    } else if (insert_mode == "no-intersection") {
		    out << YAML::LocalTag("no-intersection");
	    } else if (insert_mode == "split-intersection") {
		    out << YAML::LocalTag("split-intersection");
	    }
    }
    out << YAML::Flow << YAML::BeginSeq << edge.from() << edge.to() << YAML::EndSeq;
  }

  out << YAML::EndSeq
      << YAML::EndMap;

  std::ofstream s(filename);
  s << "%YAML 1.2" << std::endl
    << "%TAG ! tag:fawkesrobotics.org,navgraph/" << std::endl
    << "---" << std::endl
    << out.c_str();
}

} // end of namespace fawkes
