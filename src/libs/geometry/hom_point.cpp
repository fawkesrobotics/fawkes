
/***************************************************************************
 *  hom_point.cpp - Homogenous point
 *
 *  Created: Thu Sep 27 17:01:55 2007
 *  Copyright  2007-2008  Daniel Beck
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

#include "hom_point.h"
#include "hom_vector.h"
#include <cmath>
#include <cstdio>
#include <exception>

namespace fawkes {

/** @class HomPoint geometry/hom_point.h
 * A homogeneous point.
 * @author Daniel Beck
 */

/**Constructor.
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param z the z-coordinate
 */
HomPoint::HomPoint(float x, float y, float z)
  : HomCoord(x, y, z, 1.0)
{
}

/** Constructor.
 * Constructs a 2-dimensional vector from a cart_coord_2d_t struct.
 *@param coord a structure for a 2-dimensional coordinate
 */
HomPoint::HomPoint(cart_coord_2d_t coord)
  : HomCoord(coord.x, coord.y, 0.0, 1.0)
{
}

/** Constructor.
 * Constructs a 3-dimensional vector from a cart_coord_3d_t struct.
 *@param coord a structure for a 3-dimensional coordinate
 */
HomPoint::HomPoint(cart_coord_3d_t coord)
  : HomCoord(coord.x, coord.y, coord.z, 1.0)
{
}

/** Constructor.
 * @param h a HomCoord
 */
HomPoint::HomPoint(const HomCoord& h)
  : HomCoord(h)
{
  if ( 1.0 != w() )
    { 
      printf("HomPoint(const HomCoord& h): The forth component of a "
	     "homogeneous point has to be 1.0 but is %f\n", w());
      throw std::exception(); 
    }
}

/** Destructor */
HomPoint::~HomPoint()
{
}

/** Obtain distance from the point to the origin.
 * @return distance to origin
 */
float
HomPoint::distance() const
{
  float d = sqrt( x() * x() + y() * y() + z() * z() );
  return d;
}

/** Move the point by the given coordiantes.
 * @param dx x-offset
 * @param dy y-offset
 * @param dz z-offset
 * @return reference to the moved point
 */
HomPoint&
HomPoint::move(float dx, float dy, float dz)
{
  this->x() += dx;
  this->y() += dy;
  this->z() += dz;

  return *this;
}

/** Move the point to the given coordiantes.
 * @param x new x-coordinate
 * @param y new y-coordinate
 * @param z new z-coordinate
 * @return reference to the moved point
 */
HomPoint&
HomPoint::move_to(float x, float y, float z)
{
  this->x() = x;
  this->y() = y;
  this->z() = z;

  return *this;
}

/** Compute the vector between two points.                                                                                                
 * @param p the other point                                                                                                               
 * @return the vector between the two points                                                                                              
 */
HomVector
HomPoint::operator-(const HomPoint& p) const
{
  HomVector v;
  v.x( x() - p.x() );
  v.y( y() - p.y() );
  v.z( z() - p.z() );
  
  return v;
}

} // end namespace fawkes
