
/***************************************************************************
 *  astar_search.h - A colli-specific A* search implementation
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
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

#ifndef __PLUGINS_COLLI_SEARCH_ASTAR_SEARCH_H_
#define __PLUGINS_COLLI_SEARCH_ASTAR_SEARCH_H_

#include "abstract_search.h"

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CLaserOccupancyGrid;
class CAStar;
class Logger;
class Configuration;

typedef struct point_struct point_t;

/** This is the plan class.
 *  Here the plan from A* is managed and cut into small pieces.
 *    Also usable methods for managing the plan are implemented here.
 */
class CSearch: public CAbstractSearch
{
 public:

  CSearch( CLaserOccupancyGrid * occGrid , Logger* logger, Configuration* config);
  virtual ~CSearch();

  ///\brief update complete plan things
  void Update( int roboX, int roboY, int targetX, int targetY );

  ///\brief returns, if the update was successful or not.
  bool UpdatedSuccessful();

  ///\brief Get the current plan
  std::vector<point_t>* GetPlan();

  ///\brief Get the robot's position in the grid, used for the plan
  point_t GetRoboPosition();

 private:

  /** Returns the current, modified waypoint to drive to. */
  point_t CalculateLocalTarget();

  /** Adjust the waypoint if it is not the final point. */
  point_t AdjustWaypoint( const point_t &local_target );

  /** Returns the current trajectory point to drive to. */
  point_t CalculateLocalTrajectoryPoint( );

  /** Method for checking if an obstacle is between two points. */
  bool IsObstacleBetween( const point_t &a, const point_t &b, const int maxcount );



  // --------------------------------- //
  //    VARIABLES
  // --------------------------------- //

  CAStar * m_pAStar;              /**< the A* search algorithm */
  std::vector< point_t > m_vPlan; /**< the local representation of the plan */

  point_t m_RoboPosition, m_TargetPosition;
  bool m_UpdatedSuccessful;
  int cfg_search_line_allowed_cost_max; /**< the config value for the max allowed costs on the line search on the a-star result */

  fawkes::Logger* logger_;
};

} // namespace fawkes

#endif
