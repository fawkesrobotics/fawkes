
/***************************************************************************
 *  rcsoft_map_node.cpp - Node used in RCSoftMapGraph
 *
 *  Created: Tue Jun 30 09:33:03 2009 (RoboCup 2009, Graz)
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: rcsoft_map_node.cpp 2828 2009-07-06 09:11:16Z tim $
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

#include "rcsoft_map_node.h"

#include <algorithm>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class RCSoftMapNode <utils/graph/rcsoft_map_node.h>
 * RCSoft map node representation.
 * @author Tim Niemueller
 */

/** Constructor for invalid node. */
RCSoftMapNode::RCSoftMapNode()
{
  __name       = "";
  __x          = 0.0;
  __y          = 0.0;
  __children.clear();
  __properties.clear();
  __aliases.clear();
}


/** Constructor.
 * @param name name of the node
 * @param x World X coordinate of the node
 * @param y World Y position of the node
 * @param children vector of child nodes
 * @param properties vector of properties
 * @param aliases vector of aliases
 */
RCSoftMapNode::RCSoftMapNode(std::string name, float x, float y,
			     std::vector< std::string > children,
			     std::vector< std::string > properties,
			     std::vector< std::string > aliases)
{
  __name       = name;
  __x          = x;
  __y          = y;
  __children   = children,
  __properties = properties;
  __aliases    = aliases;
}


/** Constructor.
 * @param name name of the node
 * @param x World X coordinate of the node
 * @param y World Y position of the node
 * @param ori orientation of node
 */
RCSoftMapNode::RCSoftMapNode(std::string name, float x, float y, float ori)
{
  __name       = name;
  __x          = x;
  __y          = y;
}


/** Get node name.
 * @return node name
 */
const std::string &
RCSoftMapNode::name() const
{
  return __name;
}


/** Get node X coordinate.
 * @return node X coordinate
 */
float
RCSoftMapNode::x() const
{
  return __x;
}


/** Get node Y coordinate.
 * @return node Y coordinate
 */
float
RCSoftMapNode::y() const
{
  return __y;
}


/** Get children of node.
 * @return reference to vector of child node names
 */
std::vector<std::string> &
RCSoftMapNode::children()
{
  return __children;
}


/** Get properties of node.
 * @return reference to vector of properties
 */
std::vector<std::string> &
RCSoftMapNode::properties()
{
  return __properties;
}


/** Get aliases.
 * @return reference to vector of aliases
 */
std::vector<std::string> &
RCSoftMapNode::aliases()
{
  return __aliases;
}


/** Check if node has a specific property.
 * @param property property to check for
 * @return true if the node has the specified property, false otherwise
 */
bool
RCSoftMapNode::has_property(std::string property)
{
  return (std::find(__properties.begin(), __properties.end(), property) != __properties.end());
}


/** Check if node has a specific alias.
 * @param alias alias to check for
 * @return true if the node has the specified alias, false otherwise
 */
bool
RCSoftMapNode::has_alias(std::string alias)
{
  return (std::find(__aliases.begin(), __aliases.end(), alias) != __aliases.end());
}


/** Check if the node is valid.
 * @return true if the node is valid, false otherwise
 */
bool 
RCSoftMapNode::is_valid() const
{
  return (__name != "");
}

} // end of namespace fawkes
