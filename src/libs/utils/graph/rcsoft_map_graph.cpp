
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

#include "rcsoft_map_graph.h"
#include <utils/misc/string_conversions.h>
#include <core/exception.h>

#include <libxml++/libxml++.h>
#include <cmath>

using namespace xmlpp;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class RCSoftMapGraph <utils/graph/rcsoft_map_graph.h>
 * Read RCSoft map graphs.
 * This class can be used to read and search map graphs of our old software
 * framework RCSoft.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param filename path to the file to read
 */
RCSoftMapGraph::RCSoftMapGraph(std::string filename)
{
  __dom = new DomParser();
  //dom->set_validate();
  __dom->set_substitute_entities();
  __dom->parse_file(filename);
  __root = __dom->get_document()->get_root_node();
  if ( __root == NULL ) {
    throw Exception("Could not parse graph");
  }

  parse_graph();
}


/** Destructor. */
RCSoftMapGraph::~RCSoftMapGraph()
{
  delete __dom;
}


/** Get text content of a node.
 * @param root node from where to start the search
 * @param subnode optional subnode prefix, may not include the /text() suffix
 * for retrieval of the text!
 * @return text of the node
 * @exception Exception thrown if the node does not have the desired sub-node
 * or if the node is not a text node.
 */
std::string
RCSoftMapGraph::get_node_text(xmlpp::Node *root, std::string subnode)
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


/** Get a map node from a XML node.
 * @param node XML node to parse as a map node.
 * @return map node representation
 */
RCSoftMapNode
RCSoftMapGraph::get_node(xmlpp::Node *node)
{
  std::string name = get_node_text(node, "NodeName");
  float x = get_node_float(node, "x");
  float y = get_node_float(node, "y");
  std::vector<std::string> properties;
  NodeSet prop_set = node->find("Property");
  for (NodeSet::iterator i = prop_set.begin(); i != prop_set.end(); ++i) {
    properties.push_back(get_node_text(*i));
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

  return RCSoftMapNode(name, x, y, children, properties, aliases);
}

/** Get node content as a float.
 * @param root node from where to start the search
 * @param subnode optional subnode prefix, may not include the /text() suffix
 * for retrieval of the text!
 * @return text of node converted to a float
 */
float
RCSoftMapGraph::get_node_float(xmlpp::Node *root, std::string subnode)
{
  std::string s = get_node_text(root, subnode);
  return StringConversions::to_float(s);
}


/** Parse the graph. */
void
RCSoftMapGraph::parse_graph()
{
  // get graph name
  __graph_name = get_node_text(__root, "/Graph/GraphName");

  // get nodes
  NodeSet rootnode_set = __root->find("/Graph/Root/Node");
  if ( rootnode_set.size() != 1 ) {
    throw Exception("No root node defined");
  }
  __root_node = get_node(rootnode_set[0]);
  __nodes.push_back(__root_node);

  // get nodes
  NodeSet node_set = __root->find("/Graph/Node");
  if ( node_set.size() == 0 ) {
    throw Exception("No nodes defined");
  }
  for (NodeSet::iterator i = node_set.begin(); i != node_set.end(); ++i) {
    __nodes.push_back(get_node(*i));
  }
}


/** Get graph name.
 * @return name of the graph.
 */
std::string
RCSoftMapGraph::graph_name()
{
  return __graph_name;
}


/** Get root node.
 * @return root node
 */
fawkes::RCSoftMapNode
RCSoftMapGraph::root_node()
{
  return __root_node;
}


/** Get all parsed nodes.
 * @return vector of nodes
 */
std::vector<fawkes::RCSoftMapNode>
RCSoftMapGraph::nodes()
{
  return __nodes;
}


/** Get node with given name or alias.
 * @param name_or_alias name or alias to search for
 * @return node with the given name or alias, or an invalid node if the
 * node could not be found.
 */
RCSoftMapNode
RCSoftMapGraph::node(std::string name_or_alias)
{
  std::vector<fawkes::RCSoftMapNode>::iterator i;
  for (i = __nodes.begin(); i != __nodes.end(); ++i) {
    if ( (i->name() == name_or_alias) || i->has_alias(name_or_alias)) {
      return *i;
    }
  }
  return RCSoftMapNode();
}


/** Search nodes for specific property.
 * Searches all nodes and returns the ones which have the specified property.
 * @param property property to search for
 * @return vector of nodes having the desired property
 */
std::vector<fawkes::RCSoftMapNode>
RCSoftMapGraph::search_nodes(std::string property)
{
  if (property == "") {
    return nodes();
  } else {
    std::vector<fawkes::RCSoftMapNode> rv;

    std::vector<fawkes::RCSoftMapNode>::iterator i;
    for (i = __nodes.begin(); i != __nodes.end(); ++i) {
      if ( i->has_property(property) ) {
	rv.push_back(*i);
      }
    }

    return rv;
  }
}


/** Find node closest to a specified position.
 * @param pos_x X world coordinate of close point
 * @param pos_y Y world coordinate of close point
 * @param property an optional property that nodes must have to be considered
 * @return the closest node
 */
fawkes::RCSoftMapNode
RCSoftMapGraph::closest_node(float pos_x, float pos_y, std::string property)
{
  std::vector<fawkes::RCSoftMapNode> nodes = search_nodes(property);

  float min_dist = HUGE;

  std::vector<fawkes::RCSoftMapNode>::iterator i;
  std::vector<fawkes::RCSoftMapNode>::iterator elem = nodes.begin();
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
    return RCSoftMapNode();
  } else {
    return *elem;
  }
}

} // end of namespace fawkes
