/***************************************************************************
 *  timed_reservation_list_edge_constraint.cpp - edge constraint that holds a static
 *                                   list of edges and a duration to block
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

#include <plugins/navgraph/constraints/timed_reservation_list_edge_constraint.h>

#include <algorithm>

namespace fawkes{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphTimedReservationListEdgeConstraint <plugins/navgraph/constraints/timed_reservation_list_edge_constraint.h>
 * Constraint that holds a list of edges to block with timeouts.
 * @author Sebastian Reuter
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger logger used for debug logging
 * @param name name of edge constraint
 */
NavGraphTimedReservationListEdgeConstraint::NavGraphTimedReservationListEdgeConstraint(
  Logger *logger, std::string name, fawkes::Clock *clock)
  : NavGraphEdgeConstraint(name)
{
  logger_ = logger;
  clock_ = clock;
  logger_->log_info("Timed Edge Constraint", "Created %s constraint", name.c_str() );
}

/** Constructor.
 * @param logger logger used for debug logging
 * @param name name of edge constraint
 * @param edge_time_list list of edges with valid_time
 */
NavGraphTimedReservationListEdgeConstraint::NavGraphTimedReservationListEdgeConstraint(
  Logger *logger, std::string name, fawkes::Clock *clock,
  std::vector<std::pair<fawkes::TopologicalMapEdge, fawkes::Time>> edge_time_list)
  : NavGraphEdgeConstraint(name)
{
  logger_ = logger;
  clock_ = clock;
  constraint_name_ = name;
  edge_time_list_ = edge_time_list;
  logger_->log_info("Timed Edge Constraint", "Created %s constraint", name.c_str() );
}


/** Virtual empty destructor. */
NavGraphTimedReservationListEdgeConstraint::~NavGraphTimedReservationListEdgeConstraint()
{
  logger_->log_info("Timed Edge Constraint", "Deleted %s constraint", constraint_name_.c_str() );
}


/** Compute if the validity duration has passed. */
bool
NavGraphTimedReservationListEdgeConstraint::compute(void) throw()
{

  fawkes::Time now(clock_);
  std::vector<std::pair<TopologicalMapEdge, fawkes::Time>> erase_list;
  for (const std::pair<TopologicalMapEdge, fawkes::Time> &ec : edge_time_list_) {
	  if(now > ec.second){
	    erase_list.push_back(ec);
	  }
  }
  for (const std::pair<TopologicalMapEdge, fawkes::Time> &ec : erase_list) {
	  edge_time_list_.erase(std::remove(edge_time_list_.begin(), edge_time_list_.end(), ec), edge_time_list_.end());
      modified_ = true;
      logger_->log_info("Timed Edge Constraint", "Deleted edge '%s_%s' from '%s' because it validity duration ran out", ec.first.from().c_str(),ec.first.to().c_str(), name_.c_str() );
  }

  if (modified_) {
    modified_ = false;
    return true;
  } else {
    return false;
  }
}


/** Add a single edge to constraint list.
 * @param edge edge to add to constraint list
 * @param valid_time valid time for this edge
 */
void
NavGraphTimedReservationListEdgeConstraint::add_edge(const fawkes::TopologicalMapEdge &edge,
					       fawkes::Time valid_time)
{
  fawkes::Time now(clock_);

  if (valid_time < now) {
    logger_->log_warn("Timed Edge Constraint", "Constraint '%s' received node with old reservation time='%f' - now='%f'", name_.c_str(), valid_time.in_sec(), now.in_sec());
  }
  if (! has_edge(edge)) {
    modified_ = true;
    edge_time_list_.push_back(std::make_pair(edge, valid_time));
    std::string txt = edge.from(); txt +="_"; txt+=edge.to();
    //logger_->log_info("Timed Edge Constraint", "Adding Edge '%s' to constraint '%s'", txt.c_str(), name_.c_str() );
  }
}


/** Add multiple edges to constraint list.
 * @param edges edges to add to constraint list
 */
void
NavGraphTimedReservationListEdgeConstraint::add_edges(
  const std::vector<std::pair<fawkes::TopologicalMapEdge, fawkes::Time>> &edges)
{
  std::string txt = "{";
  for (const std::pair<TopologicalMapEdge, fawkes::Time> &ec : edges) {
    add_edge(ec.first, ec.second);
    txt += ec.first.from(); txt += "_"; txt += ec.first.to(); txt += ",";
  }
  txt.erase(txt.length()-1,1); txt += "}";
  //logger_->log_info("Timed Edge Constraint", "Adding Edges '%s' to constraint '%s'", txt.c_str(), name_.c_str() );
}


/** Remove a single edge from the constraint list.
 * @param edge edge to remote
 */
void
NavGraphTimedReservationListEdgeConstraint::remove_edge(const fawkes::TopologicalMapEdge &edge)
{
  std::vector<std::pair<TopologicalMapEdge, fawkes::Time>>::iterator ec
    = std::find_if(edge_time_list_.begin(), edge_time_list_.end(),
		   [&edge](const std::pair<fawkes::TopologicalMapEdge, fawkes::Time> &p) {
		     return p.first == edge;
		   });

  if (ec != edge_time_list_.end()) {
    modified_ = true;
    edge_time_list_.erase(ec);
  }
}


/** Check if constraint has a specific edge.
 * @param edge edge to check
 * @return true if edge is in list, false otherwise
 */
bool
NavGraphTimedReservationListEdgeConstraint::has_edge(const fawkes::TopologicalMapEdge &edge)
{
  return (std::find_if(edge_time_list_.begin(), edge_time_list_.end(),
		       [&edge](const std::pair<fawkes::TopologicalMapEdge, fawkes::Time> &p) {
			 return p.first == edge;
		       })
	  != edge_time_list_.end());
}

/** Check if constraint has a specific edge.
 * @param from originating-node
 * @return to destinating-node
 */
bool
NavGraphTimedReservationListEdgeConstraint::blocks(const fawkes::TopologicalMapNode &from,
		      const fawkes::TopologicalMapNode &to) throw()
{
  for (const std::pair<fawkes::TopologicalMapEdge, fawkes::Time> &te : edge_time_list_){
	  if( (( te.first.from() == from.name()) && (te.first.to() == to.name())) ||
			 ((te.first.to() == from.name()) && (te.first.from() == to.name())))
	  {
		  return true;
	  }
  }
  return false;
}


/** Get list of blocked edges.
 * @return list of blocked edges
 */
const std::vector<std::pair<fawkes::TopologicalMapEdge, fawkes::Time>> &
NavGraphTimedReservationListEdgeConstraint::edge_time_list() const
{
  return edge_time_list_;
}


/** Remove all edges. */
void
NavGraphTimedReservationListEdgeConstraint::clear_edges()
{
  if (! edge_time_list_.empty()) {
    modified_ = true;
    edge_time_list_.clear();
  }
}


} // end of namespace fawkes
