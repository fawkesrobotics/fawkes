
/***************************************************************************
 *  map_graph.h - Map graph for storing pathplan information
 *
 *  Created: Tue Jun 30 09:25:09 2009 (RoboCup 2009, Graz)
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: rcsoft_map_graph.h 2710 2009-06-30 12:47:20Z tim $
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

#ifndef __UTILS_GRAPH_RCSOFT_MAP_GRAPH_H_
#define __UTILS_GRAPH_RCSOFT_MAP_GRAPH_H_

#include "rcsoft_map_node.h"

namespace xmlpp {
  class DomParser;
  class Node;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RCSoftMapGraph
{
 public:
  RCSoftMapGraph(std::string filename);
  ~RCSoftMapGraph();
  
  std::string                         graph_name();
  std::vector<fawkes::RCSoftMapNode>  nodes();
  fawkes::RCSoftMapNode               node(std::string name_or_alias);
  fawkes::RCSoftMapNode               root_node();

  fawkes::RCSoftMapNode               closest_node(float pos_x, float pos_y,
						   std::string property);

  std::vector<fawkes::RCSoftMapNode>  search_nodes(std::string property);

 private:
  void            parse_graph();
  std::string     get_node_text(xmlpp::Node *root, std::string subnode = "");
  float           get_node_float(xmlpp::Node *root, std::string subnode = "");
  RCSoftMapNode   get_node(xmlpp::Node *node);

 private:
  xmlpp::DomParser *__dom;
  xmlpp::Node      *__root;

  fawkes::RCSoftMapNode __root_node;
  std::string  __graph_name;
  std::vector<fawkes::RCSoftMapNode> __nodes;
};

} // end of namespace fawkes

#endif
