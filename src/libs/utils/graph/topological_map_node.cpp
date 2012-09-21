
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

TopologicalMapNode::TopologicalMapNode()
{
}


TopologicalMapNode::TopologicalMapNode(std::string name, float x, float y,
                                       std::map<std::string, std::string> properties)
{
  name_ = name;
  x_ = x;
  y_ = y;
  properties_ = properties;
}


TopologicalMapNode::TopologicalMapNode(std::string name, float x, float y)
{
  name_ = name;
  x_ = x;
  y_ = y;
}

std::string
TopologicalMapNode::property(std::string prop)
{
  if (properties_.find(prop) != properties_.end()) {
    return properties_[prop];
  } else {
    return "";
  }
}


void
TopologicalMapNode::set_property(std::string property, std::string value)
{
  properties_[property] = value;
}

void
TopologicalMapNode::set_property(std::string property, float value)
{
  properties_[property] = StringConversions::to_string(value);
}

void
TopologicalMapNode::set_property(std::string property, int value)
{
  properties_[property] = StringConversions::to_string(value);
}

void
TopologicalMapNode::set_property(std::string property, bool value)
{
  properties_[property] = value ? "true" : "false";
}


void
TopologicalMapNode::set_reachable_nodes(std::vector<std::string> reachable_nodes)
{
  reachable_nodes_ = reachable_nodes;
}

} // end of namespace fawkes
