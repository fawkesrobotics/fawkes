
/***************************************************************************
 *  hom_point.cpp - Homogenous point
 *
 *  Created: Thu Sep 27 17:01:55 2007
 *  Copyright  2007-2008  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#include <geometry/hom_point.h>
#include <geometry/hom_vector.h>
#include <cmath>

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
 * @param h a HomCoord
 */
HomPoint::HomPoint(const HomCoord& h)
  : HomCoord(h)
{
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
