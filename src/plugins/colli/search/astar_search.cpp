
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

/** @class CSearch <plugins/colli/search/astar_search.h>
 * This class tries to translate the found plan to interpreteable
 * data for the rest of the program.
 */

/** Constructor. Constructs the plan, initializes an A* Object and
 *  makes a reference to the OccupancyGrid.
 * @param occGrid The laser occupancy-grid
 * @param logger The fawkes logger
 * @param config The fawkes configuration.
 */
CSearch::CSearch( CLaserOccupancyGrid * occGrid, Logger* logger, Configuration* config)
 : CAbstractSearch( occGrid, logger ),
   logger_( logger )
{
  logger_->log_debug("CSearch", "(Constructor): Entering");
  std::string cfg_prefix = "/plugins/colli/search/";
  cfg_search_line_allowed_cost_max  = config->get_int((cfg_prefix + "line/cost_max").c_str());
  m_pAStar = new CAStar( occGrid, logger, config );
  logger_->log_debug("CSearch", "(Constructor): Exiting");
}

/** Destructor */
CSearch::~CSearch()
{
  delete m_pAStar;
}

/** Perform an Update by searching in the occgrid for a plan from robopos to targetpos.
 * precondition: the occupancy grid has to be updated previously!
 * @param roboX Robot x position in grid
 * @param roboY Robot y position in grid
 * @param targetX Target x position in grid
 * @param targetY Target y position in grid
   */
void
CSearch::Update( int roboX, int roboY, int targetX, int targetY )
{
  m_UpdatedSuccessful = false;

  // check, if a position is in an obstacle
  m_RoboPosition    = point_t( roboX, roboY );
  m_LocalTarget     = point_t( roboX, roboY );
  m_LocalTrajectory = point_t( roboX, roboY );

  if ( m_pOccGrid->getProb( targetX, targetY ) == cell_costs_.occ ) {
    int stepX = 1;  // initializing to 1
    int stepY = 1;
    if ( roboX < targetX ) // if we search in the other direction, inverse it!
      stepX = -1;

    if ( roboY < targetY )
      stepY = -1;

    m_TargetPosition = m_pAStar->RemoveTargetFromObstacle( targetX, targetY, stepX, stepY );

  } else {
    m_TargetPosition = point_t( targetX, targetY );
  }

  m_pAStar->Solve( m_RoboPosition, m_TargetPosition, m_vPlan );

  if (m_vPlan.size() > 0) {
    m_UpdatedSuccessful = true;
    m_LocalTarget     = CalculateLocalTarget();
    m_LocalTarget     = AdjustWaypoint( m_LocalTarget );
    m_LocalTrajectory = CalculateLocalTrajectoryPoint();
  }
}


/** Check, if the update was successful or not.
 * precondition: update had to be called.
 * @return true, if update was successfule.
 */
bool
CSearch::UpdatedSuccessful()
{
  return m_UpdatedSuccessful;
}

/** Get the current plan
 * @return vector containing all the points in the grid along the plan
 */
std::vector<point_t>*
CSearch::GetPlan()
{
  return &m_vPlan;
}

/** Get the robot's position in the grid, used for the plan
 * @return Robot's position in the grid
 */
point_t
CSearch::GetRoboPosition()
{
  return m_RoboPosition;
}

/* **************************************************************************** */
/* **************************************************************************** */
/* *********** P R I V A T E  -   S T U F F *********************************** */
/* **************************************************************************** */
/* **************************************************************************** */



point_t
CSearch::CalculateLocalTarget()
{
  point_t target = m_RoboPosition;
  point_t prev   = m_RoboPosition;

  if( m_vPlan.size() >= 2 ) {
    for ( std::vector<point_t>::iterator it = m_vPlan.begin()+1; it != m_vPlan.end(); ++it ) {
      prev = target;
      target = *it;

      if( IsObstacleBetween( m_RoboPosition, target, cfg_search_line_allowed_cost_max ) ) {
        return prev;
      }
    }
    return point_t( m_vPlan.back() );

  } else {
    // return the current position if there is no plan.
    return m_RoboPosition;
  }
}


point_t
CSearch::AdjustWaypoint( const point_t &local_target )
{
  return local_target;
}



// forward and backward plans should no longer make a difference in
//   trajectory searching
point_t
CSearch::CalculateLocalTrajectoryPoint( )
{
  int x = m_RoboPosition.x;
  int y = m_RoboPosition.y;

  int max_occ = 10;

  if( x < m_LocalTarget.x ) {
    ++x;
    while( ( x < (int)m_pOccGrid->getWidth() )
        && ( x <= m_LocalTarget.x )
        && (!IsObstacleBetween( point_t(x, y), m_LocalTarget, max_occ ))
        && (!IsObstacleBetween( m_RoboPosition, point_t(x, y), max_occ ) ) )
    {
      ++x;
    }

    if ( x == m_LocalTarget.x && y == m_LocalTarget.y )
      return point_t( x, y );
    else
      return point_t( x-1, y );

  } else {
    --x;
    while( ( x > 0 )
        && ( x >= (int)m_LocalTarget.x )
        && (!IsObstacleBetween( point_t(x, y), m_LocalTarget, max_occ ))
        && (!IsObstacleBetween( m_RoboPosition, point_t(x, y), max_occ ) ) )
    {
      --x;
    }

    if ( (x == m_LocalTarget.x) && (y == m_LocalTarget.y) )
      return point_t( x, y );
    else
      return point_t( x+1, y );
  }
}


// checks per raytracing, if an obstacle is between two points.
bool
CSearch::IsObstacleBetween( const point_t &a, const point_t &b, const int maxcount )
{
  if (a.x == b.x && a.y == b.y)
    return false;

  int count = 0;
  float prob = 0.0;

  register int _xDirInt, _yDirInt;
  register int _actXGrid = a.x;
  int endXGrid = b.x;
  int dX = abs(endXGrid - _actXGrid);
  ( endXGrid > _actXGrid ? _xDirInt = 1 : _xDirInt = -1 );
  register int _actYGrid = a.y;
  int endYGrid = b.y;
  ( endYGrid > _actYGrid ? _yDirInt = 1 : _yDirInt = -1 );
  int dY = abs(endYGrid - _actYGrid);

  // decide whether direction is more x or more y, and run the algorithm
  if (dX > dY) {
    register int _P, _dPr, _dPru;
    _dPr  = dY<<1; // amount to increment decision if right is chosen (always)
    _dPru = _dPr - (dX<<1); // amount to increment decision if up is chosen
    _P    = _dPr - dX; // decision variable start value

    for ( ; (_actXGrid != endXGrid) && (_actYGrid != endYGrid); _actXGrid += _xDirInt ) {
      if( _actXGrid < 0 || _actXGrid > m_pOccGrid->getWidth()
       || _actYGrid < 0 || _actXGrid > m_pOccGrid->getHeight() )
      {
        return false;
      }

      prob = m_pOccGrid->getProb( _actXGrid, _actYGrid );

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
        logger_->log_warn("AStar_Search", "(line 261) ERROR IN RAYTRACER!");

      if ( count > maxcount )
        return true;

      ( ( _P > 0 ) ? _actYGrid += _yDirInt, _P += _dPru : _P += _dPr );
    }

  } else {
    register int _P, _dPr, _dPru;
    _dPr         = dX<<1; // amount to increment decision if right is chosen (always)
    _dPru        = _dPr - (dY<<1); // amount to increment decision if up is chosen
    _P           = _dPr - dY; // decision variable start value

    for ( ; (_actXGrid != endXGrid) && (_actYGrid != endYGrid); _actYGrid += _yDirInt ) {
      if( _actXGrid < 0 || _actXGrid > m_pOccGrid->getWidth()
       || _actYGrid < 0 || _actXGrid > m_pOccGrid->getHeight() )
      {
        return false;
      }

      prob = m_pOccGrid->getProb( _actXGrid, _actYGrid );

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
        logger_->log_warn("AStar_Search", "(line 295) ERROR IN RAYTRACER!");

      if ( count > maxcount )
        return true;

      ( ( _P > 0 ) ? _actXGrid += _xDirInt, _P += _dPru : _P += _dPr );
    }
  }

  return false; // there is no obstacle between those two points.
}

} // namespace fawkes
