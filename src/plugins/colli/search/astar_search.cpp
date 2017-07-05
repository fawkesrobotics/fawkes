
/***************************************************************************
 *  astar_search.cpp - A colli-specific A* search implementation
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

#include "astar_search.h"
#include "astar.h"
#include "og_laser.h"

#include <utils/math/types.h>
#include <logging/logger.h>
#include <config/config.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class search <plugins/colli/search/astar_search.h>
 * This class tries to translate the found plan to interpreteable
 * data for the rest of the program.
 */

/** Constructor. Constructs the plan, initializes an A* Object and
 *  makes a reference to the OccupancyGrid.
 * @param occ_grid The laser occupancy-grid
 * @param logger The fawkes logger
 * @param config The fawkes configuration.
 */
Search::Search( LaserOccupancyGrid * occ_grid, Logger* logger, Configuration* config)
 : AbstractSearch( occ_grid, logger ),
   logger_( logger )
{
  logger_->log_debug("search", "(Constructor): Entering");
  std::string cfg_prefix = "/plugins/colli/search/";
  cfg_search_line_allowed_cost_max_  = config->get_int((cfg_prefix + "line/cost_max").c_str());
  astar_ = new AStarColli( occ_grid, logger, config );
  logger_->log_debug("search", "(Constructor): Exiting");
}

/** Destructor */
Search::~Search()
{
  delete astar_;
}

/** Perform an update by searching in the occgrid for a plan from robopos to targetpos.
 * precondition: the occupancy grid has to be updated previously!
 * @param robo_x Robot x position in grid
 * @param robo_y Robot y position in grid
 * @param target_x Target x position in grid
 * @param target_y Target y position in grid
   */
void
Search::update( int robo_x, int robo_y, int target_x, int target_y )
{
  updated_successful_ = false;

  // check, if a position is in an obstacle
  robo_position_    = point_t( robo_x, robo_y );
  local_target_     = point_t( robo_x, robo_y );
  local_trajec_ = point_t( robo_x, robo_y );

  if ( occ_grid_->get_prob( target_x, target_y ) == cell_costs_.occ ) {
    int step_x = 1;  // initializing to 1
    int step_y = 1;
    if ( robo_x < target_x ) // if we search in the other direction, inverse it!
      step_x = -1;

    if ( robo_y < target_y )
      step_y = -1;

    target_position_ = astar_->remove_target_from_obstacle( target_x, target_y, step_x, step_y );

  } else {
    target_position_ = point_t( target_x, target_y );
  }

  astar_->solve( robo_position_, target_position_, plan_ );

  if (plan_.size() > 0) {
    updated_successful_ = true;
    local_target_     = calculate_local_target();
    local_target_     = adjust_waypoint( local_target_ );
    local_trajec_ = calculate_local_trajec_point();
  }
}


/** Check, if the update was successful or not.
 * precondition: update had to be called.
 * @return true, if update was successfule.
 */
bool
Search::updated_successful()
{
  return updated_successful_;
}

/** Get the current plan
 * @return vector containing all the points in the grid along the plan
 */
std::vector<point_t>*
Search::get_plan()
{
  return &plan_;
}

/** Get the robot's position in the grid, used for the plan
 * @return Robot's position in the grid
 */
point_t
Search::get_robot_position()
{
  return robo_position_;
}

/* **************************************************************************** */
/* **************************************************************************** */
/* *********** P R I V A T E  -   S T U F F *********************************** */
/* **************************************************************************** */
/* **************************************************************************** */



point_t
Search::calculate_local_target()
{
  point_t target = robo_position_;
  point_t prev   = robo_position_;

  if( plan_.size() >= 2 ) {
    for ( std::vector<point_t>::iterator it = plan_.begin()+1; it != plan_.end(); ++it ) {
      prev = target;
      target = *it;

      if( is_obstacle_between( robo_position_, target, cfg_search_line_allowed_cost_max_ ) ) {
        return prev;
      }
    }
    return point_t( plan_.back() );

  } else {
    // return the current position if there is no plan.
    return robo_position_;
  }
}


point_t
Search::adjust_waypoint( const point_t &local_target )
{
  return local_target;
}



// forward and backward plans should no longer make a difference in
//   trajectory searching
point_t
Search::calculate_local_trajec_point( )
{
  int x = robo_position_.x;
  int y = robo_position_.y;

  int max_occ = 10;

  if( x < local_target_.x ) {
    ++x;
    while( ( x < (int)occ_grid_->get_width() )
        && ( x <= local_target_.x )
        && (!is_obstacle_between( point_t(x, y), local_target_, max_occ ))
        && (!is_obstacle_between( robo_position_, point_t(x, y), max_occ ) ) )
    {
      ++x;
    }

    if ( x == local_target_.x && y == local_target_.y )
      return point_t( x, y );
    else
      return point_t( x-1, y );

  } else {
    --x;
    while( ( x > 0 )
        && ( x >= (int)local_target_.x )
        && (!is_obstacle_between( point_t(x, y), local_target_, max_occ ))
        && (!is_obstacle_between( robo_position_, point_t(x, y), max_occ ) ) )
    {
      --x;
    }

    if ( (x == local_target_.x) && (y == local_target_.y) )
      return point_t( x, y );
    else
      return point_t( x+1, y );
  }
}


// checks per raytracing, if an obstacle is between two points.
bool
Search::is_obstacle_between( const point_t &a, const point_t &b, const int maxcount )
{
  if (a.x == b.x && a.y == b.y)
    return false;

  int count = 0;
  float prob = 0.0;

  int _xDirInt, _yDirInt;
  int _actXGrid = a.x;
  int endXGrid = b.x;
  int dX = abs(endXGrid - _actXGrid);
  ( endXGrid > _actXGrid ? _xDirInt = 1 : _xDirInt = -1 );
  int _actYGrid = a.y;
  int endYGrid = b.y;
  ( endYGrid > _actYGrid ? _yDirInt = 1 : _yDirInt = -1 );
  int dY = abs(endYGrid - _actYGrid);

  // decide whether direction is more x or more y, and run the algorithm
  if (dX > dY) {
    int _P, _dPr, _dPru;
    _dPr  = dY<<1; // amount to increment decision if right is chosen (always)
    _dPru = _dPr - (dX<<1); // amount to increment decision if up is chosen
    _P    = _dPr - dX; // decision variable start value

    for ( ; (_actXGrid != endXGrid) && (_actYGrid != endYGrid); _actXGrid += _xDirInt ) {
      if( _actXGrid < 0 || _actXGrid > occ_grid_->get_width()
       || _actYGrid < 0 || _actXGrid > occ_grid_->get_height() )
      {
        return false;
      }

      prob = occ_grid_->get_prob( _actXGrid, _actYGrid );

      if ( prob == cell_costs_.free )
        ;
      else if ( prob == cell_costs_.occ )
        return true;
      else if ( prob == cell_costs_.far )
        ++count;
      else if ( prob == cell_costs_.mid )
        count += 2;
      else if ( prob == cell_costs_.near )
        count += 4;
      else
        logger_->log_warn("AStar_search", "(line 261) ERROR IN RAYTRACER!");

      if ( count > maxcount )
        return true;

      ( ( _P > 0 ) ? _actYGrid += _yDirInt, _P += _dPru : _P += _dPr );
    }

  } else {
    int _P, _dPr, _dPru;
    _dPr         = dX<<1; // amount to increment decision if right is chosen (always)
    _dPru        = _dPr - (dY<<1); // amount to increment decision if up is chosen
    _P           = _dPr - dY; // decision variable start value

    for ( ; (_actXGrid != endXGrid) && (_actYGrid != endYGrid); _actYGrid += _yDirInt ) {
      if( _actXGrid < 0 || _actXGrid > occ_grid_->get_width()
       || _actYGrid < 0 || _actXGrid > occ_grid_->get_height() )
      {
        return false;
      }

      prob = occ_grid_->get_prob( _actXGrid, _actYGrid );

      if ( prob == cell_costs_.free )
        ;
      else if ( prob == cell_costs_.occ )
        return true;
      else if ( prob == cell_costs_.far )
        ++count;
      else if ( prob == cell_costs_.mid )
        count += 2;
      else if ( prob == cell_costs_.near )
        count += 4;
      else
        logger_->log_warn("AStar_search", "(line 295) ERROR IN RAYTRACER!");

      if ( count > maxcount )
        return true;

      ( ( _P > 0 ) ? _actXGrid += _xDirInt, _P += _dPru : _P += _dPr );
    }
  }

  return false; // there is no obstacle between those two points.
}

} // namespace fawkes
