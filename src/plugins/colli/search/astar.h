
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

class LaserOccupancyGrid;
class Logger;
class Configuration;

typedef struct point_struct point_t;

/** Class AStar.
 *  This is an implementation of the A* search algorithm in a
 *    highly efficient way (I hope ;-).
 */
class AStarColli
{
 public:
  AStarColli( LaserOccupancyGrid * occGrid, Logger* logger, Configuration* config );
  ~AStarColli();

  /* =========================================== */
  /* ************* PUBLIC METHODS ************** */
  /* =========================================== */
  /** solves the given assignment.
   *  This starts the search for a path through the occupance grid to the
   *    target point.
   *  Performing astar search over the occupancy grid and returning the solution.
   */
  void solve( const point_t &robo_pos, const point_t &target_pos, std::vector<point_t> &solution );

  ///\brief Method, returning the nearest point outside of an obstacle.
  point_t remove_target_from_obstacle( int target_x, int target_y, int step_x, int step_y );

 private:
  /* =========================================== */
  /* ************ PRIVATE VARIABLES ************ */
  /* =========================================== */
  fawkes::Logger* logger_;

  // this is the local reference to the occupancy grid.
  LaserOccupancyGrid * occ_grid_;
  unsigned int width_;
  unsigned int height_;

  // Costs for the cells in grid
  colli_cell_cost_t cell_costs_;

  // this is the local robot position and target point.
  AStarState robo_pos_;
  AStarState target_state_;

  // This is a state vector...
  // It is for speed purposes. So I do not have to do a new each time
  //   I have to malloc a new one each time.
  std::vector< AStarState * > astar_states_;

  // maximum number of states available for a* and current index
  int max_states_;
  int astar_state_count_;

  // this is AStars openlist
  struct cmp {
    bool operator() ( AStarState * a1, AStarState * a2 ) const
    {
      return (a1->total_cost_ > a2->total_cost_);
    }
  };

  std::priority_queue< AStarState *, std::vector< AStarState * >, cmp > open_list_;

  // this is AStars closedList
  std::map< int, int > closed_list_;

  /* =========================================== */
  /* ************ PRIVATE METHODS ************** */
  /* =========================================== */

  // search with AStar through the OccGrid
  AStarState * search();

  // Calculate a unique key for a given coordinate
  int calculate_key( int x, int y );

  // Check if the state is a goal
  bool is_goal( AStarState * state );

  // Calculate heuristic for a given state
  int heuristic( AStarState * state );

  // Generate all children for a given State
  void generate_children( AStarState * father );

  // Generates a solution sequence for a given state
  void get_solution_sequence( AStarState * node, std::vector<point_t> &solution );

};

} // namespace fawkes

#endif
