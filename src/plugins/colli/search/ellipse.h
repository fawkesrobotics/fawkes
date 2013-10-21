
/***************************************************************************
 *  ellipse.h - A fast ellipse
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

#ifndef __PLUGINS_COLLI_SEARCH_ELLIPSE_H_
#define __PLUGINS_COLLI_SEARCH_ELLIPSE_H_

#include "../common/defines.h"

#include <utils/math/common.h>

#include <vector>
#include <cmath>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CFastEllipse
{
 public:
  // center 0, 0
  // construct a new ellipse with given values
  CFastEllipse( int radius_width, int radius_height, bool obstacle_increasement = true );
  ~CFastEllipse();

  // Return the occupied cells with their values
  inline const std::vector< int > GetEllipse()
  {
    return m_OccupiedCells;
  }

  inline int GetKey()
  {
    return m_Key;
  }

  inline void SetKey( int key )
  {
    m_Key = key;
  }


 private:
  // the occ cells, size is dividable through 3, 'cause:
  // [i]   = x coord,
  // [i+1] = y coord,
  // [i+2] = costs
  std::vector< int > m_OccupiedCells;

  // a unique identifier for each ellipse
  int m_Key;
};


inline
CFastEllipse::CFastEllipse( int radius_width, int radius_height, bool obstacle_increasement )
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

      /*
      if ( !obstacle_increasement ) {
        // ignore far distance obstacles
      } else {
        dist_far = sqr((float)x/(float)(radius_width+6)) +  sqr((float)y/(float)(radius_height+6));
      }
      */

      if( (dist > 1.0) && (dist_near > 1.0)
       && (dist_middle > 1.0) && (dist_far > 1.0) ) {
        // not in grid!

      } else if( (dist > 1.0) && (dist_near > 1.0)
              && (dist_middle > 1.0) && (dist_far <= 1.0) ) {
        m_OccupiedCells.push_back( x );
        m_OccupiedCells.push_back( y );
        m_OccupiedCells.push_back( (int)_COLLI_CELL_FAR_ );

      } else if( (dist > 1.0) && (dist_near > 1.0)
              && (dist_middle <= 1.0) ) {
        m_OccupiedCells.push_back( x );
        m_OccupiedCells.push_back( y );
        m_OccupiedCells.push_back( (int)_COLLI_CELL_MIDDLE_ );

      } else if( (dist > 1.0) && (dist_near <= 1.0)
              && (dist_middle <= 1.0) ) {
        m_OccupiedCells.push_back( x );
        m_OccupiedCells.push_back( y );
        m_OccupiedCells.push_back( (int)_COLLI_CELL_NEAR_ );

      } else if( (dist <= 1.0) && (dist_near <= 1.0)
              && (dist_middle <= 1.0) ) {
        m_OccupiedCells.push_back( x );
        m_OccupiedCells.push_back( y );
        m_OccupiedCells.push_back( (int)_COLLI_CELL_OCCUPIED_ );
      }
    }
  }
}

inline
CFastEllipse::~CFastEllipse()
{
  m_OccupiedCells.clear();
}

} // namespace fawkes

#endif
