/***************************************************************************
 *  timed_reservation_list_edge_constraint.h - edge constraint that holds a static
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

#ifndef __NAVGRAPH_CONSTRAINTS_TIMED_RESERVATION_LIST_EDGE_CONSTRAINT_H_
#define __NAVGRAPH_CONSTRAINTS_TIMED_RESERVATION_LIST_EDGE_CONSTRAINT_H_

#include <navgraph/constraints/static_list_edge_constraint.h>

#include <vector>
#include <string>

#include <utils/time/time.h>
#include <navgraph/navgraph.h>
#include <logging/logger.h>

namespace fawkes{


class NavGraphTimedReservationListEdgeConstraint : public NavGraphEdgeConstraint
{
 public:
   NavGraphTimedReservationListEdgeConstraint(Logger *logger, std::string constraint_name, fawkes::Clock *clock);

   NavGraphTimedReservationListEdgeConstraint(Logger *logger, std::string constraint_name, fawkes::Clock *clock,
		   std::vector<std::pair<fawkes::NavGraphEdge, fawkes::Time>> edge_time_list );

  virtual ~NavGraphTimedReservationListEdgeConstraint();

  const std::vector<std::pair<fawkes::NavGraphEdge, fawkes::Time>> &  edge_time_list() const;

  void add_edge(const fawkes::NavGraphEdge &edge, const fawkes::Time valid_time);
  void add_edges(const std::vector<std::pair<fawkes::NavGraphEdge, fawkes::Time>> &edges);
  void remove_edge(const fawkes::NavGraphEdge &edge);
  void clear_edges();
  bool has_edge(const fawkes::NavGraphEdge &edge);

  virtual bool compute(void) throw();
  virtual bool blocks(const fawkes::NavGraphNode &from,
 		      const fawkes::NavGraphNode &to) throw();

 private:
  std::vector<std::pair<fawkes::NavGraphEdge, fawkes::Time>> edge_time_list_;
  bool modified_;
  Logger *logger_;
  fawkes::Clock *clock_;
  std::string constraint_name_;

};

} // end namespace fawkes

#endif

