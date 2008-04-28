
/***************************************************************************
 *  utils.h - utils
 *
 *  Created: ???
 *  Copyright  2008  Volker Krause <volker.krause@rwth-aachen.de>
 *
 *  $Id: pipeline_thread.cpp 1049 2008-04-24 22:40:12Z beck $
 *
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

#ifndef __FIREVISION_APPS_OMNI_LOCALIZER_UTILS_H_
#define __FIREVISION_APPS_OMNI_LOCALIZER_UTILS_H_

#include <fvutils/base/types.h>

#include <algorithm>
#include <cmath>
#include <iostream>

inline std::ostream& operator<<( std::ostream& s, const f_point_t &pos )
{
  s << "(" << pos.x << "," << pos.y << ")";
  return s;
}

inline std::ostream& operator<<( std::ostream& s, const field_pos_t &pos )
{
  s << "(" << pos.x << "," << pos.y << ") ori: " << pos.ori;
  return s;
}

inline std::ostream& operator<<( std::ostream& s, const polar_coord_t &pos )
{
  s << "(phi:" << pos.phi << " r:" << pos.r << ")";
  return s;
}

/**
  Find value in the container given by @p first and @p last which is closest
  to @p value.
*/
template<typename T, typename InputIterator> inline T find_closest( InputIterator first, InputIterator last, T value )
{
  if ( last == first )
    return T();
  InputIterator closestIt = std::lower_bound( first, last, value );
  if ( closestIt == last ) {
    --closestIt;
    return *closestIt;
  }
  if ( closestIt == first )
    return *closestIt;
  const T closest = *closestIt;
  --closestIt;
  if ( std::abs( value - closest ) > std::abs( value - (*closestIt) ) )
    return *closestIt;
  return closest;
}

#endif
