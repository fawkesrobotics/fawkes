/***************************************************************************
 *  node_constraint.h - base class for nod constraints
 *
 *  Created: Sun Mar 02 10:47:35 2014
 *  Copyright  2014  Sebastian Reuter
 *             2014  Tim Niemueller
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __NAVGRAPH_CONSTRAINTS_NODE_CONSTRAINT_H_
#define __NAVGRAPH_CONSTRAINTS_NODE_CONSTRAINT_H_

#include <vector>
#include <string>

#include <utils/graph/topological_map_graph.h>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;

class NavGraphNodeConstraint
{
 public:
  NavGraphNodeConstraint(Logger *logger, std::string name);

  NavGraphNodeConstraint(Logger *logger, std::string name,
			 std::vector<fawkes::TopologicalMapNode> node_list);

  virtual ~NavGraphNodeConstraint();

  std::string name();
  const std::vector<fawkes::TopologicalMapNode> &  node_list() const;

  void add_node(fawkes::TopologicalMapNode &node);
  void add_nodes(std::vector<fawkes::TopologicalMapNode> &nodes);
  void remove_node(fawkes::TopologicalMapNode &node);
  void clear_nodes();

  bool has_node(fawkes::TopologicalMapNode &node);

 private:
  Logger *logger_;

 protected:
  std::vector<fawkes::TopologicalMapNode> node_list_;
  std::string name_;

};

} // end namespace fawkes

#endif
