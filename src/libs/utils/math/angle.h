
/***************************************************************************
 *  angle.h - angle related math helper functions
 *
 *  Created: Wed Jul 13 16:51:46 2005 (from FireVision)
 *  Copyright  2005-2008  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __UTILS_MATH_ANGLE_H_
#define __UTILS_MATH_ANGLE_H_

#include <cmath>

namespace fawkes {


/** Convert an angle given in degrees to radians.
 * @param deg original value in degrees
 * @return converted value in radians
 */
inline float
deg2rad(float deg)
{
  return (deg * M_PI / 180.f);
}


/** Convert an angle given in radians to degrees.
 * @param rad original value in radians
 * @return converted value in degrees
 */
inline float
rad2deg(float rad)
{
  return (rad * 180.f / M_PI);
}


/** Get distance between two 2D cartesian coordinates.
 * @param x1 X coordinate of first point
 * @param y1 Y coordinate of first point
 * @param x2 X coordinate of second point
 * @param y2 Y coordinate of second point
 */
inline float
distance(float x1, float y1, float x2, float y2)
{
  return sqrt( (x2-x1) * (x2-x1) + (y2-y1) * (y2-y1) );
}

/** Normalize angle in radian between -PI and PI.
 * The given angle in radians is taken as an angle on the unit circle.
 * It is then normalized into the range -PI and PI, such that it is the
 * exact same angle on the unit circle but in the usual angle range.
 * @param angle_rad original value
 * @return normalized angle
 */
inline float 
normalize_mirror_rad(float angle_rad)
{
  if ( (angle_rad < -M_PI) || (angle_rad > M_PI) ) {
    return ( angle_rad - 2 * M_PI * round(angle_rad / (2 * M_PI)) );
  } else {
    return angle_rad;
  }
}

/** Normalize angle in radian between 0 and 2*PI.
 * The given angle in radians is taken as an angle on the unit circle.
 * It is then normalized into the range 0 and 2*PI, such that it is the
 * exact same angle on the unit circle but in the usual angle range.
 * @param angle_rad original value
 * @return normalized angle
 */
inline float
normalize_rad(float angle_rad)
{
  if ( (angle_rad < 0) || (angle_rad > 2 * M_PI) ) {
    return angle_rad - 2 * M_PI * floor(angle_rad / (M_PI * 2));
  } else {
    return angle_rad;
  }
}


/** Normalizes angle in radian between -3*PI and 3*PI.
 * If the angle is above 2*PI or below 2*PI the angle will be clipped.
 * The largest full amount of (-)2*PI is subtracted, such that only the amount
 * within the range [-2*PI, 2*PI] remains. Then (-)2*PI is added again.
 * @param angle_rad original value
 * @return normalized angle
 */
inline float
normalize_bigmirror_rad(float angle_rad)
{
  if ( (angle_rad < -2*M_PI) || (angle_rad > 2*M_PI) ) {
    return (normalize_mirror_rad(angle_rad) + copysign(2*M_PI, angle_rad) );
  } else {
    return angle_rad;
  }
}


/** Determines the distance between two angle provided as radians. 
 * Quadrants of the angles are considered to determine really the minimal
 * angle difference.
 * @param angle_rad1 first angle in radian
 * @param angle_rad2 first angle in radian
 * @return distance between the two angles
 */
inline float 
angle_distance(float angle_rad1,
	       float angle_rad2)
{
  if(angle_rad2 > angle_rad1)
    {
      return angle_rad2 - angle_rad1 < M_PI ? angle_rad2 - angle_rad1 : - 2.0 * M_PI + angle_rad2 - angle_rad1; 
    }
  else
    {
      return angle_rad1 - angle_rad2 < M_PI ? angle_rad2 - angle_rad1 : 2.0 * M_PI - angle_rad1 + angle_rad2;   
    }
}


} // end namespace fawkes

#endif
