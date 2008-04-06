
/***************************************************************************
 *  hom_vector.cpp - Homogenous vector
 *
 *  Created: Wed Sep 26 17:14:08 2007
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

#include <geometry/hom_vector.h>
#include <cmath>

/** @class HomVector geometry/hom_vector.h
 * A homogeneous vector.
 * @author Daniel Beck
 */

/**Constructor.
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param z the z-coordinate
 */
HomVector::HomVector(float x, float y, float z) 
  : HomCoord(x, y, z, 0.0)
{
}

/** Constructor.
 * @param h a HomCoord
 */
HomVector::HomVector(const HomCoord& h)
  : HomCoord(h)
{
}

/** Destructor. */
HomVector::~HomVector()
{
}

/** Calculates the length of the vector
 * @return the length
 */
float
HomVector::length() const
{
  float length = sqrt( x() * x() + y() * y() + z() * z() );

  return length;
}

/** Brings the vector to unit-length.
 * @return a reference to itself
 */
HomVector&
HomVector::unit()
{
  set_length(1.0);

  return *this;
}

/** Scales the vector such that it has the given length.
 * @param length the new length
 * @return reference to a vector with given length
 */
HomVector&
HomVector::set_length(float length)
{
  float cur_len = this->length();
  if (cur_len != 0.0)
    {
      x() = x() / cur_len * length;
      y() = y() / cur_len * length;
      z() = z() / cur_len * length;
    }

  return *this;
}

/** Scale the length of the vector by the given factor.
 * @param factor the scaling factor
 */
HomVector&
HomVector::scale(float factor)
{
  x() = x() * factor;
  y() = y() * factor;
  z() = z() * factor;

  return *this;
}
