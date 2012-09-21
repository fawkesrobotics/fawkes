
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

TopologicalMapEdge::TopologicalMapEdge()
{
  directed_ = false;
}


TopologicalMapEdge::TopologicalMapEdge(std::string from, std::string to,
                                       std::map<std::string, std::string> properties,
                                       bool directed)
{
  from_ = from;
  to_ = to;
  properties_ = properties;
  directed_ = directed;
}


TopologicalMapEdge::TopologicalMapEdge(std::string from, std::string to, bool directed)
{
  from_ = from;
  to_ = to;
  directed_ = directed;
}

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
