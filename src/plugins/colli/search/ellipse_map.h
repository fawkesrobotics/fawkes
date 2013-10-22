
/***************************************************************************
 *  ellipse_map.h - Collection of fast ellipses
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

#ifndef __PLUGINS_COLLI_SEARCH_ELLIPSE_MAP_H_
#define __PLUGINS_COLLI_SEARCH_ELLIPSE_MAP_H_

#include "ellipse.h"

#include <vector>
#include <map>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CEllipseMap <plugins/colli/search/ellipse_map.h>
 * This is an implementation of a collection of fast ellipses.
 */

class CEllipseMap
{
 public:
  CEllipseMap( );
  ~CEllipseMap() { m_mEllipses.clear(); }

  const std::vector< int > GetEllipse( int width, int height, bool obstacle_increasement = true );

 private:
  std::map< unsigned int, CFastEllipse * > m_mEllipses;

};

/** Constructor. */
inline
CEllipseMap::CEllipseMap()
{
  //   for ( unsigned int x = 0; x < (unsigned int)max_radius_width; x++ )
  //     {
  //       for ( unsigned int y = 0; y < (unsigned int)max_radius_height; y++ )
  //  {
  //    CFastEllipse * ellipse = new CFastEllipse( x, y, robocup_mode );

  //    // What it does is the following: x * 2^16 + y. This is unique,
  //    //   because first it does a bit shift for 16 bits, and adds (or)
  //    //   afterwards a number that is smaller tham 16 bits!
  //    unsigned int key = (x << 16) | y;
  //    ellipse->SetKey( key );
  //    m_mEllipses[ key ] = ellipse;
  //  }
  //     }
}

/** Get the occupied cells that match a given ellipse.
 * @param width The width of the ellipse
 * @param height The height of the ellipse
 * @param obstacle_increasement Enable obstacle increasement?
 * @return vector with pairwise cell coordinates (x,y), that are occupied by such an ellipse
 */
inline const std::vector< int >
CEllipseMap::GetEllipse( int width, int height, bool obstacle_increasement )
{
  unsigned int key = ((unsigned int)width << 16) | (unsigned int)height;

  std::map< unsigned int, CFastEllipse * >::iterator p = m_mEllipses.find( key );
  if ( p == m_mEllipses.end() ) {
    // ellipse nicht gefunden!
    CFastEllipse * ellipse = new CFastEllipse( width, height, obstacle_increasement );
    ellipse->SetKey( key );
    m_mEllipses[ key ] = ellipse;
    return ellipse->GetEllipse();

  } else {
    // ellipse in p gefunden
    return m_mEllipses[ key ]->GetEllipse();
  }
}

} // namespace fawkes

#endif
