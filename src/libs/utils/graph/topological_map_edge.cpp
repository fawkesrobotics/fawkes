
/***************************************************************************
 *  topological_map_edge.cpp - Topological graph
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

#include <utils/graph/topological_map_edge.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class TopologicalMapEdge <utils/graph/topological_map_edge.h>
 * Topological graph edge.
 * @author Tim Niemueller
 */

/** Constructor for an invalid edge. */
TopologicalMapEdge::TopologicalMapEdge()
{
  directed_ = false;
}


/** Constructor.
 * @param from originating node name
 * @param to target node name
 * @param properties properties of the new node
 * @param directed true if the edge is directed, false for bidirectional edges
 */
TopologicalMapEdge::TopologicalMapEdge(std::string from, std::string to,
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
TopologicalMapEdge::TopologicalMapEdge(std::string from, std::string to, bool directed)
{
  from_ = from;
  to_ = to;
  directed_ = directed;
}

/** Set originating node name.
 * @param from originating node name
 */
void
TopologicalMapEdge::set_from(std::string from)
{
  from_ = from;
}


/** Set target node name.
 * @param to target node name
 */
void
TopologicalMapEdge::set_to(std::string to)
{
  to_ = to;
}


/** Set directed state.
 * @param directed true if the edge is directed, false for bidirectional edges
 */
void
TopologicalMapEdge::set_directed(bool directed)
{
  directed_ = directed;
}

/** Get specified property as string.
 * @param prop property key
 * @return property value as string
 */
std::string
TopologicalMapEdge::property(std::string prop)
{
  if (properties_.find(prop) != properties_.end()) {
    return properties_[prop];
  } else {
    return "";
  }
}

} // end of namespace fawkes
