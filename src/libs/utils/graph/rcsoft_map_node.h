
/***************************************************************************
 *  rcsoft_map_node.h - Node used in RCSoftMapGraph
 *
 *  Created: Tue Jun 30 09:27:08 2009 (RoboCup 2009, Graz)
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: rcsoft_map_node.h 2826 2009-07-06 08:59:01Z tim $
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

#ifndef __UTILS_GRAPH_RCSOFT_MAP_NODE_H_
#define __UTILS_GRAPH_RCSOFT_MAP_NODE_H_

#include <string>
#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RCSoftMapNode
{
 public:
  RCSoftMapNode();
  RCSoftMapNode(std::string name, float x, float y,
		std::vector<std::string> children,
		std::vector<std::string> properties,
		std::vector<std::string> aliases);


  const std::string &        name() const;
  float                      x() const;
  float                      y() const;

  std::vector<std::string> & properties();
  std::vector<std::string> & aliases();
  std::vector<std::string> & children();

  bool has_property(std::string property);
  bool has_alias(std::string property);
  bool is_valid() const;

 private:
  std::string __name;
  float       __x;
  float       __y;
  std::vector<std::string> __children;
  std::vector<std::string> __properties;
  std::vector<std::string> __aliases;
};

} // end of namespace fawkes

#endif
