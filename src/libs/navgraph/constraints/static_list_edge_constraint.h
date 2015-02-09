/***************************************************************************
 *  static_list_edge_constraint.h - edge constraint that holds a static list
 *                                  of edges to block
 *
 *  Created: Sat Jul 12 16:46:50 2014
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

#ifndef __NAVGRAPH_CONSTRAINTS_STATIC_LIST_EDGE_CONSTRAINT_H_
#define __NAVGRAPH_CONSTRAINTS_STATIC_LIST_EDGE_CONSTRAINT_H_

#include <navgraph/constraints/edge_constraint.h>

#include <vector>
#include <string>

#include <navgraph/navgraph.h>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphStaticListEdgeConstraint : public NavGraphEdgeConstraint
{
 public:
  NavGraphStaticListEdgeConstraint(std::string name);

  NavGraphStaticListEdgeConstraint(std::string name,
				   std::vector<fawkes::NavGraphEdge> &edge_list);

  virtual ~NavGraphStaticListEdgeConstraint();

  const std::vector<fawkes::NavGraphEdge> &  edge_list() const;

  void add_edge(const fawkes::NavGraphEdge &edge);
  void add_edges(const std::vector<fawkes::NavGraphEdge> &edges);
  void remove_edge(const fawkes::NavGraphEdge &edge);
  void clear_edges();
  bool has_edge(const fawkes::NavGraphEdge &edge);

  virtual bool compute(void) throw();

  virtual bool blocks(const fawkes::NavGraphNode &from,
		      const fawkes::NavGraphNode &to) throw();

 private:
  std::vector<fawkes::NavGraphEdge> edge_list_;
  bool modified_;

};

} // end namespace fawkes

#endif
