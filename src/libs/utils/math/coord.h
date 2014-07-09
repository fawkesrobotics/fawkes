
/***************************************************************************
 *  coord.h - coordinate transformation and coord sys related functions
 *
 *  Created: Wed Jul 16 19:07:37 2008 (RoboCup 2008, Suzhou)
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_MATH_COORD_H_
#define __UTILS_MATH_COORD_H_

#include <cmath>

namespace fawkes {


/** Convert a 2D cartesian coordinate to a 2D polar coordinate.
 * @param polar_phi Phi of the polar coordinate
 * @param polar_dist distnace of the polar coordinate
 * @param cart_x upon return contains X of the cartesian coordinate
 * @param cart_y upon return contains Y of the cartesian coordinate
 */
inline void
cart2polar2d(float cart_x, float cart_y,
	     float *polar_phi, float *polar_dist)
{
  *polar_phi  = atan2f(cart_y, cart_x);
  *polar_dist = sqrtf(cart_x * cart_x + cart_y * cart_y);
}

/** Convert a 3D cartesian coordinate (x, y, z) to a 3D polar coordinate.
 * @param cart_x      in
 * @param cart_y      in
 * @param cart_z      in
 * @param polar_phi   out
 * @param polar_theta out
 * @param polar_r     out
 */
inline void
cart2polar3d( float cart_x, float cart_y, float cart_z,
              float& polar_phi, float& polar_theta, float& polar_r)
{
  polar_r      = sqrtf( cart_x*cart_x + cart_y*cart_y + cart_z*cart_z );
  polar_phi    = atan2f( cart_y, cart_x );
  polar_theta  = -1.0 * atan2f( cart_z, sqrtf( cart_x*cart_x + cart_y*cart_y ) );
}

/** Convert a 2D polar coordinate to a 2D cartesian coordinate.
 * @param polar_phi Phi of the polar coordinate
 * @param polar_dist distnace of the polar coordinate
 * @param cart_x upon return contains X of the cartesian coordinate
 * @param cart_y upon return contains Y of the cartesian coordinate
 */
inline void
polar2cart2d(float polar_phi, float polar_dist,
	     float *cart_x, float *cart_y)
{
  *cart_x = polar_dist * cosf(polar_phi);
  *cart_y = polar_dist * sinf(polar_phi);
}


} // end namespace fawkes

#endif
