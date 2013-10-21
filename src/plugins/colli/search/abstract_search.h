
/***************************************************************************
 *  abstract_search.h - An abstract class for a search in an occupancy grid
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

#ifndef __PLUGINS_COLLI_SEARCH_ABSTRACTSEARCH_H_
#define __PLUGINS_COLLI_SEARCH_ABSTRACTSEARCH_H_

#include <logging/logger.h>
#include <utils/math/types.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CLaserOccupancyGrid;

/** This is the abstract search interpretation class for an arbitrary
 * search algorithm to find its way through
 * an Occupancy grid from a robopos to a targetpos.
 */
class CAbstractSearch
{
 public:
  ///
  CAbstractSearch( CLaserOccupancyGrid * occGrid, Logger* logger );

  ///
  virtual ~CAbstractSearch();

  /** update complete plan things
   *  precondition: the occupancy grid has to be updated previously!
   */
  virtual void Update( int roboX, int roboY, int targetX, int targetY ) = 0;

  /** Returns after an update, if the update was successful.
   */
  virtual bool UpdatedSuccessful() = 0;

  /** return pointer to the local target. do not modify afterwards
   *  precondition: Update has to be called before this is ok here
   */
  const point_t& GetLocalTarget();

  /** return pointer to the local trajectory point. do not modify afterwards
   *  precondition: Update has to be called before this is ok here
   */
  const point_t& GetLocalTrajec();

 protected:
  // the occupancy grid
  CLaserOccupancyGrid * m_pOccGrid;

  // the calculated information where to drive to
  point_t m_LocalTarget, m_LocalTrajectory;
};




inline
CAbstractSearch::CAbstractSearch( CLaserOccupancyGrid * occGrid, Logger* logger )
{
  logger->log_info("CAbstractSearch", "(Constructor): Entering");
  m_pOccGrid = occGrid;
  logger->log_info("CAbstractSearch", "(Constructor): Exiting");
}

inline
CAbstractSearch::~CAbstractSearch()
{
}
inline const point_t&
CAbstractSearch::GetLocalTarget()
{
  return m_LocalTarget;
}


inline const point_t&
CAbstractSearch::GetLocalTrajec()
{
  return m_LocalTrajectory;
}

} // namespace fawkes

#endif
