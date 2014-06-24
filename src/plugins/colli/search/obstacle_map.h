
/***************************************************************************
 *  obstacle_map.h - Collection of fast obstacles
 *
 *  Created: Wed Apr 30 16:03:23 2014
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

#ifndef __PLUGINS_COLLI_SEARCH_OBSTACLE_MAP_H_
#define __PLUGINS_COLLI_SEARCH_OBSTACLE_MAP_H_

#include "obstacle.h"
#include "../common/types.h"

#include <vector>
#include <map>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ColliObstacleMap <plugins/colli/search/obstacle_map.h>
 * This is an implementation of a collection of fast obstacles.
 */

class ColliObstacleMap
{
 public:
  ColliObstacleMap(colli_cell_cost_t cell_costs, bool is_rectangle = false);
  ~ColliObstacleMap() { obstacles_.clear(); }

  const std::vector< int > get_obstacle( int width, int height, bool obstacle_increasement = true );

 private:
  std::map< unsigned int, ColliFastObstacle * > obstacles_;
  bool is_rectangle_;
  colli_cell_cost_t cell_costs_;
};

/** Constructor.
 * @param cell_costs struct containing the occ-grid cell costs
 * @param is_rectangle Defines if obstacles are rectangles or ellipses(=default).
 */
inline
ColliObstacleMap::ColliObstacleMap(colli_cell_cost_t cell_costs, bool is_rectangle)
{
  cell_costs_ = cell_costs;
  is_rectangle_ = is_rectangle;
}

/** Get the occupied cells that match a given obstacle.
 * @param width The width of the obstacle
 * @param height The height of the obstacle
 * @param obstacle_increasement Enable obstacle increasement?
 * @return vector with pairwise cell coordinates (x,y), that are occupied by such an obstacle
 */
inline const std::vector< int >
ColliObstacleMap::get_obstacle( int width, int height, bool obstacle_increasement )
{
  unsigned int key = ((unsigned int)width << 16) | (unsigned int)height;

  std::map< unsigned int, ColliFastObstacle * >::iterator p = obstacles_.find( key );
  if ( p == obstacles_.end() ) {
    // obstacle not found
    ColliFastObstacle* obstacle;
    if( is_rectangle_ )
      obstacle = new ColliFastRectangle( width, height, cell_costs_ );
    else
      obstacle = new ColliFastEllipse( width, height, cell_costs_, obstacle_increasement );
    obstacle->set_key( key );
    obstacles_[ key ] = obstacle;
    return obstacle->get_obstacle();

  } else {
    // obstacle found in p (previously created obstacles)
    return obstacles_[ key ]->get_obstacle();
  }
}

} // namespace fawkes

#endif
