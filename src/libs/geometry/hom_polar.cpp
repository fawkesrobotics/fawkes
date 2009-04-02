
/***************************************************************************
 *  hom_polar.cpp - A polar coordinate
 *
 *  Created: Tue April 22 22:55:26 2008
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
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

#include <geometry/hom_polar.h>
#include <geometry/hom_transform.h>
#include <cmath>
#include <cstdio>

namespace fawkes {

/** @class HomPolar <geometry/hom_polar.h>
 * A homogeneous representation of a polar coordinate.
 * @author Daniel Beck
 */

/** Constructor (two-dimensional).
 * @param r the radius
 * @param phi the rotation around the z-axis
 */
HomPolar::HomPolar(float r, float phi)
  : HomVector(r, 0.0, 0.0)
{
  m_r   = r;
  m_phi_z = phi;
  m_phi_y = 0.0;

  HomCoord::rotate_z(phi);
}

/** Constructor (three-dimensional).
 * @param r the radius
 * @param phi_z the rotation around the z-axis
 * @param phi_y the rotation around the new y-axis (after rotating around the z-axis)
 */
HomPolar::HomPolar(float r, float phi_z, float phi_y)
  : HomVector(r, 0.0, 0.0)
{
  m_r     = r;
  m_phi_z = phi_z;
  m_phi_y = phi_y;
  
  HomCoord::rotate_z(m_phi_z);
  HomCoord::rotate_y(m_phi_y);
}

/** Copy constructor.
 * @param h a HomCoord
 */
HomPolar::HomPolar(const HomCoord& h)
  : HomVector(h) 
{
  m_r     = sqrt( x() * x() + y() * y() + z() * z() );
  m_phi_z = atan2f(y(), x());;
  m_phi_y = atan2f(z(), sqrt( x() * x() + y() * y() ) );
}

/** Desctructor. */
HomPolar::~HomPolar()
{
}

/** Obtain the radius.
 * @return the radius
 */
float
HomPolar::r() const
{
  return m_r;
}

/** Set the radius.
 * @param r the new radius
 */
void
HomPolar::r(float r)
{
  if ( x() == 0.0 && y() == 0.0 && z() == 0.0 )
    {
      x() = 1.0;
      rotate_z(m_phi_z);
      rotate_y(m_phi_y);
    }
     
  set_length(r);
  m_r = r;
}

/** Get the rotation angle around the z-axis.
 * @return the rotation angle around the z-axis
 */
float
HomPolar::phi() const
{
  return m_phi_z;
}

/** Set the rotation angle around the z-axis.
 * @param phi the rotation angle around the z-axis
 */
void
HomPolar::phi(float phi)
{
  float phi_y = m_phi_y;

  x() = m_r;
  y() = 0.0;
  z() = 0.0;

  HomTransform t;
  t.rotate_z(phi);
  t.rotate_y(m_phi_y);
  
  *this = t * (*this);

  m_phi_z = phi;
  m_phi_y = phi_y;
}

/** Get the rotation angle around the z-axis.
 * @return the rotation angle around the z-axis
 */
float
HomPolar::phi_z() const
{
  return m_phi_z;
}

/** Set the rotation angle around the z-axis.
 * @param phi_z the rotation angle around the z-axis
 */
void
HomPolar::phi_z(float phi_z)
{
  float phi_y = m_phi_y;

  x() = m_r;
  y() = 0.0;
  z() = 0.0;

  HomTransform t;
  t.rotate_z(phi_z);
  t.rotate_y(phi_y);
  
  *this = t * (*this);

  m_phi_z = phi_z;
  m_phi_y = phi_y;
}

/** Obtain the rotation angle around the y-axis after rotating around the z-axis.
 * @return the rotation angle around the y-axis
 */
float
HomPolar::phi_y() const
{
  return m_phi_y;
}

/** Set the rotation angle around the y-axis after rotating around the z-axis.
 * @param phi_y the new rotation angle around the y-axis
 */
void
HomPolar::phi_y(float phi_y)
{
  float phi_z = m_phi_z;
  x() = m_r;
  y() = 0.0;
  z() = 0.0;

  HomTransform t;
  t.rotate_z(phi_z);
  t.rotate_y(phi_y);
  
  *this = t * (*this);

  m_phi_z = phi_z;
  m_phi_y = phi_y;
}

/** Set both rotation angles.
 * @param phi_z the rotation angle around the z-axis
 * @param phi_y the rotation angle around the y-axis
 */
void
HomPolar::phi(float phi_z, float phi_y)
{
  x() = m_r;
  y() = 0.0;
  z() = 0.0;

  HomTransform t;
  t.rotate_z(phi_z);
  t.rotate_y(phi_y);
  
  *this = t * (*this);

  m_phi_z = phi_z;
  m_phi_y = phi_y;
}

HomPolar&
HomPolar::rotate_x(float rad)
{
  HomCoord::rotate_x(rad);
  
  m_phi_z = atan2f(y(), x());
  m_phi_y = atan2f(z(), sqrt( x() * x() + y() * y() ) );

  return *this;
}

HomPolar&
HomPolar::rotate_y(float rad)
{
  HomCoord::rotate_y(rad);
  
  m_phi_z = atan2f(y(), x());
  m_phi_y = atan2f(z(), sqrt( x() * x() + y() * y() ) );

  return *this;
}

HomPolar&
HomPolar::rotate_z(float rad)
{
  HomCoord::rotate_z(rad);
  m_phi_z += rad;

  return *this;
}

/** Substraction operator.
 * The result of subtracting two polar positions from each other is another polar
 * position that represent the cartesian vector which is the result of subtracting
 * the corresponding cartesian vectors from each other.
 * @param p another polar position
 * @return the result of the substraction
 */
HomPolar
HomPolar::operator-(const HomPolar& p) const
{
  HomPolar ret = HomPolar( HomCoord::operator-(p) );

  ret.m_phi_z = atan2f(ret.y(), ret.x());
  ret.m_phi_y = atan2f(ret.z(), sqrt( ret.x() * ret.x() + ret.y() * ret.y() ) );
  
  return ret;
}

/** Subtraction-assignment operator.
 * @param p the other polar position
 * @return reference of the result
 */
HomPolar&
HomPolar::operator-=(const HomPolar& p)
{
  *this = *this - p;

  return *this;
}

/** Addition operator.
 * The result of adding two polar positions from each other is another polar
 * position that represent the cartesian vector which is the result of adding
 * the corresponding cartesian vectors to each other.
 * @param p another polar position
 * @return the result of the substraction
 */
HomPolar
HomPolar::operator+(const HomPolar& p) const
{
  HomPolar ret = HomPolar( HomCoord::operator+(p) );

  ret.m_phi_z = atan2f(ret.y(), ret.x());
  ret.m_phi_y = atan2f(ret.z(), sqrt( ret.x() * ret.x() + ret.y() * ret.y() ) );
  
  return ret;
}

/** Addition-assignment operator.
 * @param p the other polar position
 * @return reference of the result
 */
HomPolar&
HomPolar::operator+=(const HomPolar& p)
{
  *this = *this + p;

  return *this;
}

/** Assignemnt operator.
 * @param p the other polar position
 * @return reference of the result
 */
HomPolar&
HomPolar::operator=(const HomPolar& p)
{
  HomCoord::operator=(p);
  
  m_r     = p.m_r;
  m_phi_z = p.m_phi_z;
  m_phi_y = p.m_phi_y;

  return *this;
}

/** Convert the polar coordinate to a cartesian coordinate.
 * @return the cartesian coordinate
 */
HomVector
HomPolar::get_vector() const
{
  HomVector v;
  v.x() = x();
  v.y() = y();
  v.z() = z();

  return v;
}

} // end namespace fawkes
