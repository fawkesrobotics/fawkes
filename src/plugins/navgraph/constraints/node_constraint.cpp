/***************************************************************************
 *  node_constraint.cpp - base class for nod constraints
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

#include <plugins/navgraph/constraints/node_constraint.h>

#include <logging/logger.h>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** Constructor.
 * @param logger logger for debugging
 * @param name name of node constraint
 */
NavGraphNodeConstraint::NavGraphNodeConstraint(Logger *logger, std::string name)
{
  logger_ = logger;
  name_   = name;
}



/** Constructor.
 * @param logger logger for debugging
 * @param name name of node constraint
 * @param node_list list of nodes to block
 */
NavGraphNodeConstraint::NavGraphNodeConstraint(Logger *logger, std::string name,
					       std::vector<fawkes::TopologicalMapNode> node_list)
{
  logger_    = logger;
  name_      = name;
  node_list_ = node_list;
}

/** Virtual empty destructor. */
NavGraphNodeConstraint::~NavGraphNodeConstraint()
{
}


/** Add a single node to constraint list.
 * @param node node to add to constraint list
 */
void
NavGraphNodeConstraint::add_node(fawkes::TopologicalMapNode &node)
{
  if (! has_node(node)) {
    node_list_.push_back(node);
    logger_->log_info("abstract_node_constraint", "Added Node %s to '%s'.",
		     node.name().c_str(), name_.c_str() );
  } else {
    logger_->log_info("abstract_node_constraint", "Node %s is already in '%s'",
		     node.name().c_str(), name_.c_str() );
  }
}

/** Add multiple nodes to constraint list.
 * @param nodes nodes to add to constraint list
 */
void
NavGraphNodeConstraint::add_nodes(std::vector<fawkes::TopologicalMapNode> &nodes)
{
  for (unsigned int i = 0; i < nodes.size(); i++) {
    logger_->log_info("abstract_node_constraint", "Added Node %s to '%s'",
		      nodes[i].name().c_str(), name_.c_str() );
    add_node(nodes[i]);
  }
}

/** Remove a single node from the constraint list.
 * @param node node to remote
 */
void
NavGraphNodeConstraint::remove_node(fawkes::TopologicalMapNode &node)
{
  if( ! node_list_.empty()) {
    std::vector<TopologicalMapNode>::iterator i;
    for( i = node_list_.begin(); i != node_list_.end(); ++i){
      if( i->name() == node.name() ){
	node_list_.erase(i);
	logger_->log_info("abstract_node_constraint",
			  "removed node %s from constraint node_list_", node.name().c_str() );
      }
    }
  }

  logger_->log_error("abstract_node_constraint",
		     "Tried to remove not existing node %s from constraint list",
		     node.name().c_str() );
}

/** Check if constraint has a specific node.
 * @param node node to check
 */
bool
NavGraphNodeConstraint::has_node(fawkes::TopologicalMapNode &node)
{
  if ( !node_list_.empty()) {

    std::vector<TopologicalMapNode>::const_iterator i;
    for( i = node_list_.begin(); i != node_list_.end(); ++i){
      if( i->name() == node.name() ){
	return true;
      }
    }
  }
  return false;
}

/** Get name of constraint.
 * @return name of constraint
 */
std::string
NavGraphNodeConstraint::name()
{
  return name_;
}

const std::vector<fawkes::TopologicalMapNode> &
NavGraphNodeConstraint::node_list() const
{
  return node_list_;
}

void
NavGraphNodeConstraint::clear_nodes()
{
  node_list_.clear();
}


} // end of namespace fawkes
