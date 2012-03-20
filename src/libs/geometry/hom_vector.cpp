
/***************************************************************************
 *  hom_vector.cpp - Homogenous vector
 *
 *  Created: Wed Sep 26 17:14:08 2007
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

#include <geometry/hom_vector.h>
#include <cmath>
#include <cstdio>
#include <exception>

namespace fawkes {

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
  if ( 0.0 != w() )
    { 
      printf("HomVector(const HomCoord& h): The fourth component of a "
	     "homogeneous vector has to be 0.0 but it is %f\n", w()); 
      throw std::exception();
    }
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
  return sqrt( length_square() );
}

/** Calculates the squared length length of the vector (faster than length
 * @return the squared length
 */
float
HomVector::length_square() const
{
  return x() * x() + y() * y() + z() * z();
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
  if (this->length() == 0.0) return *this;

  float scale_factor = length / this->length();

  x() = x() * scale_factor;
  y() = y() * scale_factor;
  z() = z() * scale_factor;

  return *this;
}

/** Compute the angle between two vectors.
 * @param v the other vector
 * @return the angle (-M_PI ... M_PI)
 */
float
HomVector::angle_xy(const HomVector& v) const
{
  if ( 0.0 == length() || 0.0 == v.length() )
  { return 0.0; }

  float a = atan2f(v.y(), v.x()) - atan2f(y(), x());

  if ( a > M_PI ) { a -= 2 * M_PI; }
  else if ( a < -M_PI ) { a += 2 * M_PI; }

  return a;
}

} // end namespace fawkes
