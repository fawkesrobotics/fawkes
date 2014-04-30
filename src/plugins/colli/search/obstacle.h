
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

#include "../common/defines.h"

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
  // the occ cells, size is dividable through 3, 'cause:
  // [i]   = x coord,
  // [i+1] = y coord,
  // [i+2] = costs
  std::vector< int > occupied_cells_;

 private:
  // a unique identifier for each obstacle
  int key_;
};

class ColliFastRectangle : public ColliFastObstacle
{
 public:
  ColliFastRectangle(int width, int height);
};

class ColliFastEllipse : public ColliFastObstacle
{
 public:
  ColliFastEllipse(int width, int height, bool obstacle_increasement = true);
};


/** Constructor for FastRectangle.
 * @param width radius width of the new rectangle
 * @param height radius height of the new rectangle
 */
inline
ColliFastRectangle::ColliFastRectangle( int width, int height )
{
  int y_start = -width/2;
  int x_start = -height/2;

  //consider (0,0) to be bottom-left corner of obstacle.
  for( int x=-6; x<height + 6; ++x ) {
    for( int y=-6; y<width + 6; ++y ) {
      occupied_cells_.push_back( x_start + x );
      occupied_cells_.push_back( y_start + y );

      if( x < -4 || x >= height+4 || y < -4 || y >= width+4 ) {
        occupied_cells_.push_back( (int)_COLLI_CELL_FAR_ );

      } else if( x < -2 || x >= height+2 || y < - 2 || y >= width+2 ) {
        occupied_cells_.push_back( (int)_COLLI_CELL_MIDDLE_ );

      } else if( x < 0 || x >= height || y < 0 || y >= width ) {
        occupied_cells_.push_back( (int)_COLLI_CELL_NEAR_ );

      } else {
        occupied_cells_.push_back( (int)_COLLI_CELL_OCCUPIED_ );
      }

      ++y_start;
    }

    ++x_start;
  }
}

/** Constructor for FastEllipse.
 * @param radius_width radius width of the new ellipse
 * @param radius_height radius height of the new ellipse
 * @param obstacle_increasement Increase obstacles?
 */
inline
ColliFastEllipse::ColliFastEllipse( int radius_width, int radius_height, bool obstacle_increasement )
{
  float dist = 1000.0;
  float dist_near = 1000.0;
  float dist_middle = 1000.0;
  float dist_far = 1000.0;

  int maxRad = std::max( radius_width, radius_height );

  for( int y = -(maxRad+6); y <= (maxRad+6); y++ ) {
    for( int x = -(maxRad+6); x <= (maxRad+6); x++ ) {
      dist        = sqr((float)x/(float)radius_width)     + sqr((float)y/(float)radius_height);
      dist_near   = sqr((float)x/(float)(radius_width+2)) + sqr((float)y/(float)(radius_height+2));
      dist_middle = sqr((float)x/(float)(radius_width+4)) + sqr((float)y/(float)(radius_height+4));


      //if ( !obstacle_increasement ) {
      //  // ignore far distance obstacles
      //} else {
      //  dist_far = sqr((float)x/(float)(radius_width+6)) +  sqr((float)y/(float)(radius_height+6));
      //}


      if( (dist > 1.0) && (dist_near > 1.0)
       && (dist_middle > 1.0) && (dist_far > 1.0) ) {
        // not in grid!

      } else if( (dist > 1.0) && (dist_near > 1.0)
              && (dist_middle > 1.0) && (dist_far <= 1.0) ) {
        occupied_cells_.push_back( x );
        occupied_cells_.push_back( y );
        occupied_cells_.push_back( (int)_COLLI_CELL_FAR_ );

      } else if( (dist > 1.0) && (dist_near > 1.0)
              && (dist_middle <= 1.0) ) {
        occupied_cells_.push_back( x );
        occupied_cells_.push_back( y );
        occupied_cells_.push_back( (int)_COLLI_CELL_MIDDLE_ );

      } else if( (dist > 1.0) && (dist_near <= 1.0)
              && (dist_middle <= 1.0) ) {
        occupied_cells_.push_back( x );
        occupied_cells_.push_back( y );
        occupied_cells_.push_back( (int)_COLLI_CELL_NEAR_ );

      } else if( (dist <= 1.0) && (dist_near <= 1.0)
              && (dist_middle <= 1.0) ) {
        occupied_cells_.push_back( x );
        occupied_cells_.push_back( y );
        occupied_cells_.push_back( (int)_COLLI_CELL_OCCUPIED_ );
      }
    }
  }
}


} // namespace fawkes

#endif
