
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

#include "og_laser.h"
#include "../common/types.h"

#include <logging/logger.h>
#include <utils/math/types.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CAbstractSearch <plugins/colli/search/abstract_search.h>
 * This is the abstract search interpretation class for an arbitrary
 * search algorithm to find its way through
 * an Occupancy grid from a robopos to a targetpos.
 */

class CAbstractSearch
{
 public:

  CAbstractSearch( CLaserOccupancyGrid * occGrid, Logger* logger );
  virtual ~CAbstractSearch();

  /** update complete plan things
   *  precondition: the occupancy grid has to be updated previously!
   * @param roboX Robot x position in grid
   * @param roboY Robot y position in grid
   * @param targetX Target x position in grid
   * @param targetY Target y position in grid
   */
  virtual void Update( int roboX, int roboY, int targetX, int targetY ) = 0;

  /** Checks if the update was successful.
   * @return true if "Update(...)" was successful, fals otherwise.
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
  CLaserOccupancyGrid * m_pOccGrid; /**< The occupancy grid */

  point_t m_LocalTarget;      /**< the calculated target where to drive to */
  point_t m_LocalTrajectory;  /**< the calculated trajectory where to drive to */

  colli_cell_cost_t cell_costs_; /**< The costs for cells in occupancy grid */
};



/** Constructor.
 * @param occGrid The laser occupancy-grid
 * @param logger The fawkes logger
 */
inline
CAbstractSearch::CAbstractSearch( CLaserOccupancyGrid * occGrid, Logger* logger )
{
  logger->log_debug("CAbstractSearch", "(Constructor): Entering");
  m_pOccGrid = occGrid;
  cell_costs_ = m_pOccGrid->get_cell_costs();
  logger->log_debug("CAbstractSearch", "(Constructor): Exiting");
}

/** Destructor. */
inline
CAbstractSearch::~CAbstractSearch()
{
}

/** Get the local target in the grid.
 * @return The local target in grid as a point_t struct
 */
inline const point_t&
CAbstractSearch::GetLocalTarget()
{
  return m_LocalTarget;
}

/** Get the local trajectory in the grid.
 * @return The local trajectory in grid as a point_t struct
 */
inline const point_t&
CAbstractSearch::GetLocalTrajec()
{
  return m_LocalTrajectory;
}

} // namespace fawkes

#endif
