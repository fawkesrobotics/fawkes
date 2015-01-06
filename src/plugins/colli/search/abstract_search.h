
/***************************************************************************
 *  abstract_search.h - An abstract class for a search in an occupancy grid
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013-2014  Bahram Maleki-Fard
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

/** @class AbstractSearch <plugins/colli/search/abstract_search.h>
 * This is the abstract search interpretation class for an arbitrary
 * search algorithm to find its way through
 * an Occupancy grid from a robopos to a targetpos.
 */

class AbstractSearch
{
 public:
  AbstractSearch( LaserOccupancyGrid * occ_grid, Logger* logger );
  virtual ~AbstractSearch();

  /** update complete plan things
   *  precondition: the occupancy grid has to be updated previously!
   * @param robo_x Robot x position in grid
   * @param robo_y Robot y position in grid
   * @param target_x Target x position in grid
   * @param target_y Target y position in grid
   */
  virtual void update( int robo_x, int robo_y, int target_x, int target_y ) = 0;

  /** Checks if the update was successful.
   * @return true if "update(...)" was successful, fals otherwise.
   */
  virtual bool updated_successful() = 0;

  /** return pointer to the local target. do not modify afterwards
   *  precondition: update has to be called before this is ok here
   */
  const point_t& get_local_target();

  /** return pointer to the local trajectory point. do not modify afterwards
   *  precondition: update has to be called before this is ok here
   */
  const point_t& get_local_trajec();

 protected:
  LaserOccupancyGrid * occ_grid_; /**< The occupancy grid */

  point_t local_target_;  /**< the calculated target where to drive to */
  point_t local_trajec_;  /**< the calculated trajectory where to drive to */

  colli_cell_cost_t cell_costs_; /**< The costs for cells in occupancy grid */
};



/** Constructor.
 * @param occ_grid The laser occupancy-grid
 * @param logger The fawkes logger
 */
inline
AbstractSearch::AbstractSearch( LaserOccupancyGrid * occ_grid, Logger* logger )
{
  logger->log_debug("AbstractSearch", "(Constructor): Entering");
  occ_grid_ = occ_grid;
  cell_costs_ = occ_grid_->get_cell_costs();
  logger->log_debug("AbstractSearch", "(Constructor): Exiting");
}

/** Destructor. */
inline
AbstractSearch::~AbstractSearch()
{
}

/** Get the local target in the grid.
 * @return The local target in grid as a point_t struct
 */
inline const point_t&
AbstractSearch::get_local_target()
{
  return local_target_;
}

/** Get the local trajectory in the grid.
 * @return The local trajectory in grid as a point_t struct
 */
inline const point_t&
AbstractSearch::get_local_trajec()
{
  return local_trajec_;
}

} // namespace fawkes

#endif
