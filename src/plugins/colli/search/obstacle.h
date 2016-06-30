
/***************************************************************************
 *  obstacle.h - A fast obstacle
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

#ifndef __PLUGINS_COLLI_SEARCH_OBSTACLE_H_
#define __PLUGINS_COLLI_SEARCH_OBSTACLE_H_

#include "../common/types.h"
#include <utils/math/common.h>

#include <vector>
#include <cmath>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ColliFastObstacle <plugins/colli/search/obstacle.h>
 * This is an implementation of a a fast obstacle.
 */
class ColliFastObstacle
{
 public:
  ~ColliFastObstacle()
  {
    occupied_cells_.clear();
  }

  /** Return the occupied cells with their values
   * @return vector containing the occupied cells (alternating x and y coordinates)
   */
  inline const std::vector< int > get_obstacle()
  {
    return occupied_cells_;
  }

  /** Get the key
   * @return The key
   */
  inline int get_key() {
    return key_;
  }

  /** Set key.
   * @param key the new key
   */
  inline void set_key( int key ) {
    key_ = key;
  }

 protected:
  /** Aligned array of the occ cells, size is dividable through 3, because:
   * [i]   = x coord,
   * [i+1] = y coord,
   * [i+2] = costs
   */
  std::vector< int > occupied_cells_;

 private:
  // a unique identifier for each obstacle
  int key_;
};

/** @class ColliFastRectangle
 * This is an implementation of a a fast rectangle.
 */
class ColliFastRectangle : public ColliFastObstacle
{
 public:
  ColliFastRectangle(int width, int height, colli_cell_cost_t &costs);
};

/** @class ColliFastEllipse
 * This is an implementation of a a fast ellipse.
 */
class ColliFastEllipse : public ColliFastObstacle
{
 public:
  ColliFastEllipse(int width, int height, colli_cell_cost_t &costs, bool obstacle_increasement = true);
};


/** Constructor for FastRectangle.
 * @param width radius width of the new rectangle
 * @param height radius height of the new rectangle
 * @param costs struct containing the occ-grid cell costs
 */
inline
ColliFastRectangle::ColliFastRectangle( int width, int height, colli_cell_cost_t &costs )
{
  int y_start = -width/2;
  int x_start = -height/2;

  //consider (0,0) to be bottom-left corner of obstacle.
  for( int x=-3; x<height + 3; ++x ) {
    for( int y=-3; y<width + 3; ++y ) {
      occupied_cells_.push_back( x_start + x );
      occupied_cells_.push_back( y_start + y );

      if( x < -2 || x >= height+2 || y < -2 || y >= width+2 ) {
        occupied_cells_.push_back( costs.far );

      } else if( x < -1 || x >= height+1 || y < - 1 || y >= width+1 ) {
        occupied_cells_.push_back( costs.mid );

      } else if( x < 0 || x >= height || y < 0 || y >= width ) {
        occupied_cells_.push_back( costs.near );

      } else {
        occupied_cells_.push_back( costs.occ );
      }
    }
  }
}

/** Constructor for FastEllipse.
 * @param width radius width of the new ellipse
 * @param height radius height of the new ellipse
 * @param costs struct containing the occ-grid cell costs
 * @param obstacle_increasement Increase obstacles?
 */
inline
ColliFastEllipse::ColliFastEllipse( int width, int height, colli_cell_cost_t &costs, bool obstacle_increasement )
{
  float dist = 1000.f;
  float dist_near = 1000.f;
  float dist_middle = 1000.f;
  float dist_far = 1000.f;

  int radius_width  = round(width/2.f);
  int radius_height = round(height/2.f);

  int maxRad = std::max( radius_width, radius_height );

  for( int y = -(maxRad+8); y <= (maxRad+8); y++ ) {
    for( int x = -(maxRad+8); x <= (maxRad+8); x++ ) {
      dist        = sqr((float)y/(float)radius_width)     + sqr((float)x/(float)radius_height);
      dist_near   = sqr((float)y/(float)(radius_width+2)) + sqr((float)x/(float)(radius_height+2));
      dist_middle = sqr((float)y/(float)(radius_width+4)) + sqr((float)x/(float)(radius_height+4));
      dist_far    = sqr((float)x/(float)(radius_width+8)) +  sqr((float)y/(float)(radius_height+8));

      if( (dist > 1.f) && (dist_near > 1.f)
       && (dist_middle > 1.f) && (dist_far > 1.f) ) {
        // not in grid!

      } else if( (dist > 1.f) && (dist_near > 1.f)
              && (dist_middle > 1.f) && (dist_far <= 1.f) ) {
        occupied_cells_.push_back( x );
        occupied_cells_.push_back( y );
        occupied_cells_.push_back( costs.far );

      } else if( (dist > 1.f) && (dist_near > 1.f)
              && (dist_middle <= 1.f) ) {
        occupied_cells_.push_back( x );
        occupied_cells_.push_back( y );
        occupied_cells_.push_back( costs.mid );

      } else if( (dist > 1.f) && (dist_near <= 1.f)
              && (dist_middle <= 1.f) ) {
        occupied_cells_.push_back( x );
        occupied_cells_.push_back( y );
        occupied_cells_.push_back( costs.near );

      } else if( (dist <= 1.f) && (dist_near <= 1.f)
              && (dist_middle <= 1.f) ) {
        occupied_cells_.push_back( x );
        occupied_cells_.push_back( y );
        occupied_cells_.push_back( costs.occ );
      }
    }
  }
}


} // namespace fawkes

#endif
