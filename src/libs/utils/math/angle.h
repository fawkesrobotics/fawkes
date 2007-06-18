
/***************************************************************************
 *  angle.h - angle related math helper functions
 *
 *  Generated: Wed Jul 13 16:51:46 2005 (from FireVision)
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_MATH_H_
#define __UTILS_MATH_H_

#include <cmath>

#include <stdio.h>

inline float
deg2rad(float deg)
{
  return (deg * M_PI / 180.f);
}


inline float
rad2deg(float rad)
{
  return (rad * 180.f / M_PI);
}


inline float
distance(float x1, float y1, float x2, float y2)
{
  return sqrt( (x2-x1) * (x2-x1) + (y2-y1) * (y2-y1) );
}

// normalize a rad between -PI and PI
inline float 
normalize_mirror_rad(float angle_rad)
{
  if ( (angle_rad < -M_PI) || (angle_rad > M_PI) ) {
    return ( angle_rad - 2 * M_PI * round(angle_rad / (2 * M_PI)) );
  } else {
    return angle_rad;
  }
}

inline float
normalize_rad(float angle_rad)
{
  if ( (angle_rad < 0) || (angle_rad > 2 * M_PI) ) {
    return angle_rad - 2 * M_PI * round(angle_rad / (M_PI * 2));
  } else {
    return angle_rad;
  }
}

#endif

