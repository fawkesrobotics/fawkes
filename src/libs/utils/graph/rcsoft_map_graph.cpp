
/***************************************************************************
 *  rcosoft_map_graph.cpp - Map graph for storing pathplan information
 *
 *  Created: Tue Jun 30 09:43:38 2009 (RoboCup 2009, Graz)
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: rcsoft_map_graph.cpp 2826 2009-07-06 08:59:01Z tim $
 *
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

#include <utils/graph/rcsoft_map_graph.h>
#include <utils/graph/topological_map_graph.h>
#include <utils/misc/string_conversions.h>
#include <core/exception.h>

#include <libxml++/libxml++.h>
#include <algorithm>
#include <cmath>

using namespace xmlpp;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** Get text content of a node.
 * @param root node from where to start the search
 * @param subnode optional subnode prefix, may not include the /text() suffix
 * for retrieval of the text!
 * @return text of the node
 * @exception Exception thrown if the node does not have the desired sub-node
 * or if the node is not a text node.
 */
static std::string
get_node_text(xmlpp::Node *root, std::string subnode = "")
{
  std::string sntext;
  if (subnode == "") {
    sntext = "text()";
  } else {
    sntext = subnode + "/text()";
  }

  NodeSet set = root->find(sntext);
  if ( set.size() == 0 ) {
    throw Exception("No %s sub-node for node %s", subnode.c_str(), root->get_name().c_str());
  }
  const TextNode *value_node = dynamic_cast<const TextNode *>(set[0]);
  if ( ! value_node ) {
    throw Exception("Not a text node at %s sub-node of node %s", subnode.c_str(),
		    root->get_name().c_str());
  }

  std::string s = value_node->get_content();
  return StringConversions::trim(s);
}

/** Get node content as a float.
 * @param root node from where to start the search
 * @param subnode optional subnode prefix, may not include the /text() suffix
 * for retrieval of the text!
 * @return text of node converted to a float
 */
static float
get_node_float(xmlpp::Node *root, std::string subnode)
{
  std::string s = get_node_text(root, subnode);
  return StringConversions::to_float(s);
}


/** Get a map node from a XML node.
 * @param node XML node to parse as a map node.
 * @return map node representation
 */
static TopologicalMapNode
get_node(xmlpp::Node *node)
{
  std::string name = get_node_text(node, "NodeName");
  float x = get_node_float(node, "x");
  float y = get_node_float(node, "y");
  std::map<std::string, std::string> properties;
  NodeSet prop_set = node->find("Property");
  for (NodeSet::iterator i = prop_set.begin(); i != prop_set.end(); ++i) {
    properties[get_node_text(*i)] = "";
  }

  std::vector<std::string> aliases;
  NodeSet alias_set = node->find("Alias");
  for (NodeSet::iterator i = alias_set.begin(); i != alias_set.end(); ++i) {
    aliases.push_back(get_node_text(*i));
  }

  std::vector<std::string> children;
  NodeSet child_set = node->find("Child");
  for (NodeSet::iterator i = child_set.begin(); i != child_set.end(); ++i) {
    children.push_back(get_node_text(*i));
  }

  TopologicalMapNode n(name, x, y, properties);
  n.set_reachable_nodes(children);

  return n;
}



/** Load topological map graph stored in RCSoft format.
 * @param filename path to the file to read
 * @return topological map graph read from file
 * @exception Exception thrown on any error to read the graph file
 */
TopologicalMapGraph *
load_rcsoft_graph(std::string filename)
{
  DomParser *dom = new DomParser();
  //dom->set_validate();
  dom->set_substitute_entities();
  dom->parse_file(filename);
  Node *root = dom->get_document()->get_root_node();
  if ( root == NULL ) {
    throw Exception("Could not parse graph");
  }

  // get graph name
  std::string graph_name = get_node_text(root, "/Graph/GraphName");

  // get nodes
  NodeSet rootnode_set = root->find("/Graph/Root/Node");
  if ( rootnode_set.size() != 1 ) {
    throw Exception("No root node defined");
  }
  TopologicalMapNode root_node = get_node(rootnode_set[0]);

  TopologicalMapGraph *graph = new TopologicalMapGraph(graph_name);

  graph->add_node(root_node);
  graph->set_root(root_node.name());

  // get nodes
  NodeSet node_set = root->find("/Graph/Node");
  if ( node_set.size() == 0 ) {
    throw Exception("No nodes defined");
  }
  for (NodeSet::iterator i = node_set.begin(); i != node_set.end(); ++i) {
    graph->add_node(get_node(*i));
  }

  typedef std::multimap<const std::string, const std::string> ConnMap;
  ConnMap conns;
  std::vector<TopologicalMapNode> nodes = graph->nodes();
  for (size_t i = 0; i < nodes.size(); ++i) {
    const std::vector<std::string> children = nodes[i].reachable_nodes();
    for (unsigned int j = 0; j < children.size(); ++j) {
      std::pair<ConnMap::iterator, ConnMap::iterator>
        ret = conns.equal_range(children[j]);

      ConnMap::value_type v = std::make_pair(children[j], nodes[i].name());
      ConnMap::iterator f = std::find(ret.first, ret.second, v);

      // Edge does not exist, yet, and the child node exists
      if (f == ret.second && graph->node_exists(children[j])) {
        graph->add_edge(TopologicalMapEdge(nodes[i].name(), children[j]));
      }
    }
  }

  return graph;
}

} // end of namespace fawkes
