
/***************************************************************************
 *  navgraph_edge.cpp - Topological graph
 *
 *  Created: Fri Sep 21 16:11:50 2012
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

#include <navgraph/navgraph_edge.h>

#include <core/exception.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphEdge <navgraph/navgraph_edge.h>
 * Topological graph edge.
 * @author Tim Niemueller
 */

/** Constructor for an invalid edge. */
NavGraphEdge::NavGraphEdge()
{
  directed_ = false;
}


/** Constructor.
 * @param from originating node name
 * @param to target node name
 * @param properties properties of the new node
 * @param directed true if the edge is directed, false for bidirectional edges
 */
NavGraphEdge::NavGraphEdge(std::string from, std::string to,
                                       std::map<std::string, std::string> properties,
                                       bool directed)
{
  from_ = from;
  to_ = to;
  properties_ = properties;
  directed_ = directed;
}


/** Constructor.
 * @param from originating node name
 * @param to target node name
 * @param directed true if the edge is directed, false for bidirectional edges
 */
NavGraphEdge::NavGraphEdge(std::string from, std::string to, bool directed)
{
  from_ = from;
  to_ = to;
  directed_ = directed;
}

/** Set originating node name.
 * @param from originating node name
 */
void
NavGraphEdge::set_from(std::string from)
{
  from_ = from;
}


/** Set target node name.
 * @param to target node name
 */
void
NavGraphEdge::set_to(std::string to)
{
  to_ = to;
}


/** Set nodes.
 * @param from_node originating node
 * @param to_node target node
 */
void
NavGraphEdge::set_nodes(const NavGraphNode &from_node,
			      const NavGraphNode &to_node)
{
  if (from_node.name() != from_) {
    throw Exception("Conflicting originating node names: %s vs. %s",
		    from_node.name().c_str(), from_.c_str());
  }
  if (to_node.name() != to_) {
    throw Exception("Conflicting target node names: %s vs. %s",
		    to_node.name().c_str(), to_.c_str());
  }

  from_node_ = from_node;
  to_node_   = to_node;
}

/** Set directed state.
 * @param directed true if the edge is directed, false for bidirectional edges
 */
void
NavGraphEdge::set_directed(bool directed)
{
  directed_ = directed;
}

/** Get specified property as string.
 * @param prop property key
 * @return property value as string
 */
std::string
NavGraphEdge::property(std::string prop)
{
  if (properties_.find(prop) != properties_.end()) {
    return properties_[prop];
  } else {
    return "";
  }
}

} // end of namespace fawkes
