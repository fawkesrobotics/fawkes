
/***************************************************************************
 *  navgraph_node.cpp - Topological graph node
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

#include <navgraph/navgraph_node.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphNode <navgraph/navgraph_node.h>
 * Topological graph node.
 * @author Tim Niemueller
 */

/** Constructor for invalid node. */
NavGraphNode::NavGraphNode()
  : unconnected_(false)
{
}


/** Constructor.
 * @param name name of the node
 * @param x x coordinate in global frame of node
 * @param y y coordinate in global frame of node
 * @param properties properties for the new node
 */
NavGraphNode::NavGraphNode(const std::string &name, float x, float y,
			   std::map<std::string, std::string> properties)
  : unconnected_(false)
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
NavGraphNode::NavGraphNode(const std::string &name, float x, float y)
  : unconnected_(false)
{
  name_ = name;
  x_ = x;
  y_ = y;
}


/** Set X position.
 * @param x X coordinate in global frame for node.
 */
void
NavGraphNode::set_x(float x)
{
  x_ = x;
}

/** Set Y position.
 * @param y Y coordinate in global frame for node.
 */
void
NavGraphNode::set_y(float y)
{
  y_ = y;
}


/** Set name of node.
 * @param name new name for node
 */
void
NavGraphNode::set_name(const std::string &name)
{
  name_ = name;
}

/** Set unconnected state of the node.
 * A node must be marked as unconnected explicitly or otherwise it is an
 * error that the graph will report as an error. On other hand, unconnected
 * nodes may not have any connection. By default nodes are expected to
 * have at least one connection (behaving as though this function had been
 * called with "false").
 * @param unconnected true to make this node a unconnected node,
 * false otherwise
 */
void
NavGraphNode::set_unconnected(bool unconnected)
{
  unconnected_ = unconnected;
}


/** Get specified property as string.
 * @param prop property key
 * @return property value as string
 */
std::string
NavGraphNode::property(const std::string &prop) const
{
  std::map<std::string, std::string>::const_iterator p;
  if ((p = properties_.find(prop)) != properties_.end()) {
    return p->second;
  } else {
    return "";
  }
}


/** Overwrite properties with given ones.
 * @param properties map of properties to set
 */
void
NavGraphNode::set_properties(const std::map<std::string, std::string> &properties)
{
  properties_ = properties;
}


/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraphNode::set_property(const std::string &property, const std::string &value)
{
  properties_[property] = value;
}


/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraphNode::set_property(const std::string &property, float value)
{
  properties_[property] = StringConversions::to_string(value);
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraphNode::set_property(const std::string &property, int value)
{
  properties_[property] = StringConversions::to_string(value);
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraphNode::set_property(const std::string &property, bool value)
{
  properties_[property] = value ? "true" : "false";
}


/** Set directly reachable nodes of node.
 * @param reachable_nodes vector of directly reachable nodes
 */
void
NavGraphNode::set_reachable_nodes(std::vector<std::string> reachable_nodes)
{
  reachable_nodes_ = reachable_nodes;
}

} // end of namespace fawkes
