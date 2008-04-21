/*
    Copyright (c) 2008 Volker Krause <volker.krause@rwth-aachen.de>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/

#ifndef FVOMNILOCALIZER_UTILS_H
#define FVOMNILOCALIZER_UTILS_H

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
