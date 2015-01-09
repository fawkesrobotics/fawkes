/***************************************************************************
 *  clusters_block_constraint.h - block edges close to clusters
 *
 *  Created: Fri Jul 18 21:52:40 2014
 *  Copyright  2014  Tim Niemueller
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

#ifndef __NAVGRAPH_CLUSTERS_CLUSTERS_BLOCK_CONSTRAINT_H_
#define __NAVGRAPH_CLUSTERS_CLUSTERS_BLOCK_CONSTRAINT_H_

#include <navgraph/constraints/edge_constraint.h>

#include <list>
#include <string>

class NavGraphClustersThread; 

class NavGraphClustersBlockConstraint : public fawkes::NavGraphEdgeConstraint
{
 public:
  NavGraphClustersBlockConstraint(const char *name, NavGraphClustersThread *parent);
  virtual ~NavGraphClustersBlockConstraint();

  virtual bool compute(void) throw();
  virtual bool blocks(const fawkes::NavGraphNode &from,
		      const fawkes::NavGraphNode &to) throw();

 private:
  NavGraphClustersThread *parent_;
  std::list<std::pair<std::string, std::string>> blocked_;

};

#endif
