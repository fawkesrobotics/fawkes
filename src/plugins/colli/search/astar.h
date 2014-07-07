
/***************************************************************************
 *  astar.h - A* search implementation
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

#ifndef __PLUGINS_COLLI_SEARCH_ASTAR_H_
#define __PLUGINS_COLLI_SEARCH_ASTAR_H_

#include "astar_state.h"
#include "../common/types.h"

#include <vector>
#include <queue>
#include <map>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CLaserOccupancyGrid;
class Logger;
class Configuration;

typedef struct point_struct point_t;

/** Class AStar.
 *  This is an implementation of the A* search algorithm in a
 *    highly efficient way (I hope ;-).
 */
class CAStar
{
 public:

  CAStar( CLaserOccupancyGrid * occGrid, Logger* logger, Configuration* config );
  ~CAStar();

  /* =========================================== */
  /* ************* PUBLIC METHODS ************** */
  /* =========================================== */
  /** Solves the given assignment.
   *  This starts the search for a path through the occupance grid to the
   *    target point.
   *  Performing astar search over the occupancy grid and returning the solution.
   */
  void Solve( const point_t &RoboPos, const point_t &TargetPos, std::vector<point_t> &solution );

  ///\brief Method, returning the nearest point outside of an obstacle.
  point_t RemoveTargetFromObstacle( int targetX, int targetY, int stepX, int stepY );

 private:
  /* =========================================== */
  /* ************ PRIVATE VARIABLES ************ */
  /* =========================================== */
  fawkes::Logger* logger_;

  // this is the local reference to the occupancy grid.
  CLaserOccupancyGrid * m_pOccGrid;
  unsigned int m_Width;
  unsigned int m_Height;

  // Costs for the cells in grid
  colli_cell_cost_t cell_costs_;

  // this is the local robot position and target point.
  CAStarState m_pRoboPos;
  CAStarState m_pTargetState;

  // This is a state vector...
  // It is for speed purposes. So I do not have to do a new each time
  //   I have to malloc a new one each time.
  std::vector< CAStarState * > m_vAStarStates;

  // maximum number of states available for a* and current index
  int m_MaxStates;
  int m_AStarStateCount;

  // this is AStars openlist
  struct cmp {
    bool operator() ( CAStarState * a1, CAStarState * a2 ) const
    {
      return (a1->m_TotalCost > a2->m_TotalCost);
    }
  };

  std::priority_queue< CAStarState *, std::vector< CAStarState * >, cmp > m_pOpenList;

  // this is AStars closedList
  std::map< int, int > m_hClosedList;

  /* =========================================== */
  /* ************ PRIVATE METHODS ************** */
  /* =========================================== */

  // Search with AStar through the OccGrid
  CAStarState * Search();

  // Calculate a unique key for a given coordinate
  int CalculateKey( int x, int y );

  // Check if the state is a goal
  bool IsGoal( CAStarState * state );

  // Calculate heuristic for a given state
  int Heuristic( CAStarState * state );

  // Generate all children for a given State
  void GenerateChildren( CAStarState * father );

  // Generates a solution sequence for a given state
  void GetSolutionSequence( CAStarState * node, std::vector<point_t> &solution );

};

} // namespace fawkes

#endif
