
/***************************************************************************
 *  navgraph_path.h - Topological graph - path
 *
 *  Created: Mon Jan 12 10:50:52 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_NAVGRAPH_NAVGRAPH_PATH_H_
#define __LIBS_NAVGRAPH_NAVGRAPH_PATH_H_

#include <navgraph/navgraph_node.h>

#include <vector>
#include <string>
#include <cstdlib>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraph;

class NavGraphPath {
 public:
  class Traversal {
   friend NavGraphPath;
   public:
    Traversal();
    Traversal(const NavGraphPath *path);

    const NavGraphNode & current() const;
    const NavGraphNode & peek_next() const;
    size_t               current_index() const;
    bool                 next();
    void                 reset();
    bool                 last() const;
    size_t               remaining() const;
    float                remaining_cost() const;

    bool                 running() const;
    void                 invalidate();

    void                 set_current(size_t new_current);

    /** Get parent path the traversal belongs to.
     * @return parent path */
    const NavGraphPath &  path() const
    { return *path_; }

    /** Check if traversal is initialized.
     * @return true if traversal is initialized and can be used, false otherwise. */
    operator bool() const
    { return path_ != NULL; }

   private:
    void assert_initialized() const;

   private:
    const    NavGraphPath *path_;
    ssize_t  current_;
  };

  NavGraphPath(); 
  NavGraphPath(const NavGraph *graph, std::vector<NavGraphNode> &nodes, float cost = -1);

  void add_node(const NavGraphNode &node, float cost_from_end = 0);
  void set_nodes(const std::vector<NavGraphNode> &nodes, float cost = -1);

  const NavGraph &     graph() const;
  const NavGraphNode & goal() const;

  std::string	get_path_as_string(const char delim = ':') const;
  std::vector<std::string> get_node_names() const;

  /** Get nodes along the path.
   * @return sequence of nodes that compose the path
   */
  const std::vector<NavGraphNode> &  nodes() const
  { return nodes_; }

  /** Get nodes along the path as mutable vector.
   * Use this with caution. Modifying the nodes invalidates any
   * running traversal.
   * @return sequence of nodes that compose the path
   */
  std::vector<NavGraphNode> &  nodes_mutable()
  { return nodes_; }

  bool contains(const NavGraphNode &node) const;

  bool   empty() const;
  size_t size() const;
  void   clear();

  Traversal traversal() const;

  /** Get cost of path from start to to end.
   * The cost depends on the metrics used during path search. It's unit
   * could be arbitrary, for example distance, required travel time, or
   * some generalized number. Costs are mainly useful for comparison of
   * paths. But make sure that the very same metrics were used to
   * generate the path.
   * @return cost of the path, or a value less than zero if cost is not known
   */
  float cost() const
  { return cost_; }

  bool operator<(const NavGraphPath &p) const;
  bool operator==(const NavGraphPath &p) const;

 private:
  const NavGraph            *graph_;
  std::vector<NavGraphNode>  nodes_;
  float                      cost_;
 
};


} // end of namespace fawkes

#endif
