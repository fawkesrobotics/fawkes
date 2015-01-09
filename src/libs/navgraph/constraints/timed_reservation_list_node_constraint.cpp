/***************************************************************************
 *  timed_reservation_list_node_constraint.cpp - node constraint that holds a static
 *                                   list of nodes and a duration to block
 *
 *  Created: Sat Jul 12 16:48:23 2014
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

#include <navgraph/constraints/timed_reservation_list_node_constraint.h>

#include <algorithm>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphTimedReservationListNodeConstraint <navgraph/constraints/timed_reservation_list_node_constraint.h>
 * Constraint that holds a list of nodes to block with timeouts.
 * @author Sebastian Reuter
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger logger used for debug logging
 * @param name name of node constraint
 * @param clock time source to evaluate constraint timeouts
 */
NavGraphTimedReservationListNodeConstraint::NavGraphTimedReservationListNodeConstraint
  (Logger *logger, std::string name, fawkes::Clock *clock)
  : NavGraphNodeConstraint(name)
{
  logger_ = logger;
  clock_ = clock;
}

/** Constructor.
 * @param logger logger used for debug logging
 * @param name name of node constraint
 * @param clock time source to evaluate constraint timeouts
 * @param node_time_list list of nodes with valid_time
 */
NavGraphTimedReservationListNodeConstraint::NavGraphTimedReservationListNodeConstraint
  (Logger *logger, std::string name, fawkes::Clock *clock,
  std::vector<std::pair<fawkes::NavGraphNode, fawkes::Time>> node_time_list)
  : NavGraphNodeConstraint(name)
{
  logger_ = logger;
  clock_ = clock;
  constraint_name_ = name;
  node_time_list_ = node_time_list;
}


/** Virtual empty destructor. */
NavGraphTimedReservationListNodeConstraint::~NavGraphTimedReservationListNodeConstraint()
{
}


bool
NavGraphTimedReservationListNodeConstraint::compute(void) throw()
{

  fawkes::Time now(clock_);
  std::vector<std::pair<NavGraphNode, fawkes::Time>> erase_list;
  for (const std::pair<NavGraphNode, fawkes::Time> &ec : node_time_list_) {
    if(now > ec.second){
      erase_list.push_back(ec);
    }
  }
  for (const std::pair<NavGraphNode, fawkes::Time> &ec : erase_list) {
    node_time_list_.erase(std::remove(node_time_list_.begin(), node_time_list_.end(), ec),
			  node_time_list_.end());
    modified_ = true;
    logger_->log_debug("TimedNodeConstraint",
		       "Deleted node '%s' from '%s' because its validity duration ran out",
		       ec.first.name().c_str(), name_.c_str() );
  }

  if (modified_) {
    modified_ = false;
    return true;
  } else {
    return false;
  }
}


/** Add a single node to constraint list.
 * @param node node to add to constraint list
 * @param valid_time valid time for this node
 */
void
NavGraphTimedReservationListNodeConstraint::add_node(const fawkes::NavGraphNode &node,
						     fawkes::Time valid_time)
{
  fawkes::Time now(clock_);

  if (valid_time < now) {
    logger_->log_warn("TimedNodeConstraint",
		      "Constraint '%s' received node with old reservation time='%f' - now='%f'",
		      name_.c_str(), valid_time.in_sec(), now.in_sec());
  }
  if (! has_node(node)) {
    modified_ = true;
    node_time_list_.push_back(std::make_pair(node, valid_time));
    std::string txt = node.name();
  }
}


/** Add multiple nodes to constraint list.
 * @param timed_nodes nodes with timeout to add to constraint list
 */
void
NavGraphTimedReservationListNodeConstraint::add_nodes
  (const std::vector<std::pair<fawkes::NavGraphNode, fawkes::Time>> &timed_nodes)
{
  std::string txt = "{";
  for (const std::pair<NavGraphNode, fawkes::Time> &ec : timed_nodes) {
    add_node(ec.first, ec.second);
    txt += ec.first.name(); txt += ",";
  }
  txt.erase(txt.length()-1,1); txt += "}";
}


/** Remove a single node from the constraint list.
 * @param node node to remote
 */
void
NavGraphTimedReservationListNodeConstraint::remove_node(const fawkes::NavGraphNode &node)
{
  std::vector<std::pair<NavGraphNode, fawkes::Time>>::iterator ec
    = std::find_if(node_time_list_.begin(), node_time_list_.end(),
		   [&node](const std::pair<fawkes::NavGraphNode, fawkes::Time> &p) {
		     return p.first == node;
		   });

  if (ec != node_time_list_.end()) {
    modified_ = true;
    node_time_list_.erase(ec);
  }
}


/** Check if constraint has a specific node.
 * @param node node to check
 * @return true if node is in list, false otherwise
 */
bool
NavGraphTimedReservationListNodeConstraint::has_node(const fawkes::NavGraphNode &node)
{
  return (std::find_if(node_time_list_.begin(), node_time_list_.end(),
		       [&node](const std::pair<fawkes::NavGraphNode, fawkes::Time> &p) {
			 return p.first == node;
		       })
	  != node_time_list_.end());
}

bool
NavGraphTimedReservationListNodeConstraint::blocks(const fawkes::NavGraphNode &node) throw()
{
  for (const std::pair<fawkes::NavGraphNode, fawkes::Time> &te : node_time_list_) {
    if(te.first.name() == node.name()) {
      return true;
    }
  }
  return false;
}


/** Get list of blocked nodes.
 * @return list of blocked nodes
 */
const std::vector<std::pair<fawkes::NavGraphNode, fawkes::Time>> &
NavGraphTimedReservationListNodeConstraint::node_time_list() const
{
  return node_time_list_;
}


/** Remove all nodes. */
void
NavGraphTimedReservationListNodeConstraint::clear_nodes()
{
  if (! node_time_list_.empty()) {
    modified_ = true;
    node_time_list_.clear();
  }
}


} // end of namespace fawkes
