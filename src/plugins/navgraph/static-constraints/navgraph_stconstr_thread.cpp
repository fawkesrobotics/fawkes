/***************************************************************************
 *  navgraph_stconstr_thread.cpp - static constraints for navgraph
 *
 *  Created: Fri Jul 11 17:34:18 2014
 *  Copyright  2012-2014  Tim Niemueller [www.niemueller.de]
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

#include "navgraph_stconstr_thread.h"

#include <plugins/navgraph/constraints/static_list_node_constraint.h>

using namespace fawkes;

/** @class NavGraphStaticConstraintsThread "navgraph_stconstr_thread.h"
 * Thread to statically block certain nodes from config.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphStaticConstraintsThread::NavGraphStaticConstraintsThread()
  : Thread("NavGraphStaticConstraintsThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
NavGraphStaticConstraintsThread::~NavGraphStaticConstraintsThread()
{
}

void
NavGraphStaticConstraintsThread::init()
{
  std::vector<std::string> nodes =
    config->get_strings("/plugins/navgraph/static-constraints/nodes");

  constraint_ = new NavGraphStaticListNodeConstraint("static");

  const std::vector<TopologicalMapNode> &graph_nodes = navgraph->nodes();

  std::list<std::string> missing_nodes;
  for (std::string node_name : nodes) {
    bool found = false;
    for (const TopologicalMapNode &gnode : graph_nodes) {
      if (gnode.name() == node_name) {
	constraint_->add_node(gnode);
	found = true;
	break;
      }
    }

    if (!found) {
      missing_nodes.push_back(node_name);
    }
  }

  if (! missing_nodes.empty()) {
    std::list<std::string>::iterator n = missing_nodes.begin();
    std::string err_str = *n++;
    for (;n != missing_nodes.end(); ++n) {
      err_str += ", " + *n;
    }

    delete constraint_;
    throw Exception("Some block nodes are not in graph: %s", err_str.c_str());
  }

  constraint_repo->register_constraint(constraint_);
}

void
NavGraphStaticConstraintsThread::finalize()
{
  constraint_repo->unregister_constraint(constraint_->name());
  delete constraint_;
}

void
NavGraphStaticConstraintsThread::loop()
{
}
