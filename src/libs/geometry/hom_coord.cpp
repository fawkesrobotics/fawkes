
/***************************************************************************
 *  hom_coord.cpp - Homogeneous coordinate
 *
 *  Created: Thu Sep 27 16:21:24 2007
 *  Copyright  2007-2008  Daniel Beck
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

#include <geometry/hom_coord.h>
#include <geometry/hom_transform.h>
#include <geometry/vector.h>

#include <cstdio>
#include <iomanip>

namespace fawkes {

/** @class HomCoord geometry/hom_coord.h
 * Base class for homogeneous primitives (vector and point).
 * @author Daniel Beck
 */

/** @var HomCoord::m_vector
 * The internal data container.
 */

/** Constructor.
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param z the z-coordinate
 * @param w the w-coordinate
 */
HomCoord::HomCoord(float x, float y, float z, float w)
{
  m_vector = new Vector(4);

  m_vector->set(0, x);
  m_vector->set(1, y);
  m_vector->set(2, z);
  m_vector->set(3, w);
}

/** Copy constructor.
 * @param c another HomCoord
 */
HomCoord::HomCoord(const HomCoord& c)
{
  const Vector v = *(c.m_vector);
  m_vector = new Vector(v);
}

/** Constructor.
 * @param v a vector
 */
HomCoord::HomCoord(const Vector& v)
{
  m_vector = new Vector(v);
}

/** Destructor. */
HomCoord::~HomCoord()
{
  delete m_vector;
}

/** RO-getter for x.
 * @return the value
 */
float
HomCoord::x() const
{
  return m_vector->get(0);
}

/** RW-getter for x.
 * @return a reference to the x-element
 */
float&
HomCoord::x()
{
  float& val = m_vector->get(0);
  return val;
}

/** Setter function for x.
 * @param x the new x value
 */
HomCoord&
HomCoord::x(float x)
{
  m_vector->set(0, x);
  return *this;
}

/** RO-getter for y.
 * @return the value
 */
float
HomCoord::y() const
{
  return m_vector->get(1);
}

/** RW-getter for y.
 * @return a reference to the y-element
 */
float&
HomCoord::y()
{
  float& val = m_vector->get(1);
  return val;
}

/** Setter function for y.
 * @param y the new y value
 */
HomCoord&
HomCoord::y(float y)
{
  m_vector->set(1, y);
  return *this;
}

/** RO-getter for z.
 * @return the value
 */
float
HomCoord::z() const
{
  return m_vector->get(2);
}

/** RW-getter for z.
 * @return a reference to the z-element
 */
float&
HomCoord::z()
{
  float& val = m_vector->get(2);
  return val;
}

/** Setter function for z.
 * @param z the new z value
 */
HomCoord&
HomCoord::z(float z)
{
  m_vector->set(2, z);
  return *this;
}

/** RO-getter for w.
 * @return the value
 */
float
HomCoord::w() const
{
  return m_vector->get(3);
}

/** RW-getter for w.
 * @return a reference to the w-element
 */
float&
HomCoord::w()
{
  float& val = m_vector->get(3);
  return val;
}

/** Setter function for w.
 * @param w the new w value
 */
HomCoord&
HomCoord::w(float w)
{
  m_vector->set(3, w);
  return *this;
}

/** Convenience function to rotate the HomCoord around the x-axis.
 * @param rad the roation angle in rad
 */
HomCoord&
HomCoord::rotate_x(float rad)
{
  HomTransform t;
  t.rotate_x(rad);
  transform(t);

  return *this;
}

/** Convenience function to rotate the HomCoord around the y-axis.
 * @param rad the roation angle in rad
 */
HomCoord&
HomCoord::rotate_y(float rad)
{
  HomTransform t;
  t.rotate_y(rad);
  transform(t);

  return *this;
}

/** Convenience function to rotate the HomCoord around the z-axis.
 * @param rad the roation angle in rad
 */
HomCoord&
HomCoord::rotate_z(float rad)
{
  HomTransform t;
  t.rotate_z(rad);
  transform(t);

  return *this;
}

/** Subtraction operator.
 * @param h the rhs HomCoord
 * @return the resulting HomCoord
 */
HomCoord
HomCoord::operator-(const HomCoord& h) const
{
  Vector v = (*m_vector) - (*h.m_vector);
  HomCoord result(v);
  float w = result.w();
  result.w() = (w > 1.0) ? 1.0 : w; 
  return result;
}

/** Substraction-assignment operator.
 * @param h the rhs HomCoord
 * @return reference to the resulting HomCoord
 */
HomCoord&
HomCoord::operator-=(const HomCoord& h)
{
  (*m_vector) -= (*h.m_vector);
  float w = this->w();
  this->w() = (w > 1.0) ? 1.0 : w; 
  return *this;
}

/** Addition operator.
 * @param h the rhs HomCoord
 * @return the resulting HomCoord
 */
HomCoord
HomCoord::operator+(const HomCoord& h) const
{
  Vector v = (*m_vector) + (*h.m_vector);
  HomCoord result(v);
  float w = result.w();
  result.w() = (w > 1.0) ? 1.0 : w; 
  return result;
}

/** Addition-assignment operator.
 * @param h the rhs HomCoord
 * @return reference to the resulting HomCoord
 */
HomCoord&
HomCoord::operator+=(const HomCoord& h)
{
  (*m_vector) += (*h.m_vector);
  float w = this->w();
  this->w() = (w > 1.0) ? 1.0 : w; 
  return *this;
}


/** Assignment operator.
 * @param h the rhs HomCoord
 * @return a reference of the lhs vector (this)
 */
HomCoord&
HomCoord::operator=(const HomCoord& h)
{
  (*m_vector) = (*h.m_vector);

  return *this;
}

/** Calculates the dot product of two coords.
 * @param h the rhs HomCoord
 * @return the scalar product
 */
float
HomCoord::operator*(const HomCoord& h) const
{
  return x() * h.x() + y() * h.y() + z() * h.z();
}

/** Mulitplication operator.
 * Multiply the vector with a scalar.
 * @param s a scalar
 * @return the result of multiplying the vector with the scalar
 */
HomCoord
HomCoord::operator*(const float s) const
{
  HomCoord result;
  result.x() = x() * s;
  result.y() = y() * s;
  result.z() = z() * s;
  result.w() = w();

  return result;
}

/** Multiplication-assignment operator.
 * Multiply the vector with a scalar.
 * @param s a scalar
 * @return a reference to the modified vector (this)
 */
HomCoord&
HomCoord::operator*=(const float s)
{
  x() *= s;
  y() *= s;
  z() *= s;

  return *this;
}

/** Comparison operator.
 * @param h the other HomCoord
 * @return true if h is equal to *this, false otherwise
 */
bool
HomCoord::operator==(const HomCoord& h) const
{
  return (*m_vector == *h.m_vector) ? true : false;
}

/** Inequality operator.
 * @param h the other HomCoord
 * @return true if h is not equal to *this, false otherwise
 */
bool
HomCoord::operator!=(const HomCoord& h) const
{
  return (*m_vector == *h.m_vector) ? false : true;
}

/** Appends the components of the HomCoord to the ostream.
 * @param stream to be extended
 * @return the extended stream
 */
std::ostream&
HomCoord::print(std::ostream& stream) const
{
  return stream << "[" << x() << ", "  << y() << ", "  << z() << ", "  << w() << "]T";
}

/** Transform the vector with the given transform.
 * @param t a transform
 * @return reference to the modified vector (this)
 */
HomCoord&
HomCoord::transform(const HomTransform& t)
{
  Matrix m = t.get_matrix();
  (*m_vector) = m * (*m_vector);

  return *this;
}

} // end namespace fawkes

