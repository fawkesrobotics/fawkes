
/***************************************************************************
 *  topological_map_node.cpp - Topological graph node
 *
 *  Created: Fri Sep 21 16:11:20 2012
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

#include <utils/graph/topological_map_node.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class TopologicalMapNode <utils/graph/topological_map_node.h>
 * Topological graph node.
 * @author Tim Niemueller
 */

/** Constructor for invalid node. */
TopologicalMapNode::TopologicalMapNode()
{
}


/** Constructor.
 * @param name name of the node
 * @param x x coordinate in global frame of node
 * @param y y coordinate in global frame of node
 * @param properties properties for the new node
 */
TopologicalMapNode::TopologicalMapNode(std::string name, float x, float y,
                                       std::map<std::string, std::string> properties)
{
  name_ = name;
  x_ = x;
  y_ = y;
  properties_ = properties;
}


/** Constructor.
 * @param name name of the node
 * @param x x coordinate in global frame of node
 * @param y y coordinate in global frame of node
 */
TopologicalMapNode::TopologicalMapNode(std::string name, float x, float y)
{
  name_ = name;
  x_ = x;
  y_ = y;
}


/** Set X position.
 * @param x X coordinate in global frame for node.
 */
void
TopologicalMapNode::set_x(float x)
{
  x_ = x;
}

/** Set Y position.
 * @param y Y coordinate in global frame for node.
 */
void
TopologicalMapNode::set_y(float y)
{
  y_ = y;
}


/** Set name of node.
 * @param name new name for node
 */
void
TopologicalMapNode::set_name(std::string name)
{
  name_ = name;
}


/** Get specified property as string.
 * @param prop property key
 * @return property value as string
 */
std::string
TopologicalMapNode::property(std::string prop)
{
  if (properties_.find(prop) != properties_.end()) {
    return properties_[prop];
  } else {
    return "";
  }
}


/** Set property.
 * @param property property key
 * @param value property value
 */
void
TopologicalMapNode::set_property(std::string property, std::string value)
{
  properties_[property] = value;
}


/** Set property.
 * @param property property key
 * @param value property value
 */
void
TopologicalMapNode::set_property(std::string property, float value)
{
  properties_[property] = StringConversions::to_string(value);
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
TopologicalMapNode::set_property(std::string property, int value)
{
  properties_[property] = StringConversions::to_string(value);
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
TopologicalMapNode::set_property(std::string property, bool value)
{
  properties_[property] = value ? "true" : "false";
}


/** Set directly reachable nodes of node.
 * @param reachable_nodes vector of directly reachable nodes
 */
void
TopologicalMapNode::set_reachable_nodes(std::vector<std::string> reachable_nodes)
{
  reachable_nodes_ = reachable_nodes;
}

} // end of namespace fawkes
