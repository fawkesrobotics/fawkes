
/***************************************************************************
 *  vector.cpp - (Homogenous) vector
 *
 *  Created: Wed Sep 26 17:14:08 2007
 *  Copyright  2007  Daniel Beck
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <geometry/vector.h>
#include <geometry/transform.h>
#include <core/exception.h>

#include <math.h>

using namespace std;



/** @class Vector libs/geometry/vector.h
 * A (homogeneous) vector.
 */

/** @var Vector::mLength
 * The length of the vector.
 */

/** @var Vector::mChanged
 * Indicates whether the vector was changed and the length possibly
 * needs to be recomputed.
 */


/**Constructor.
 * @param xval the x-coordinate
 * @param yval the y-coordinate
 * @param zval the z-coordinate
 */
Vector::Vector(float xval, float yval, float zval)
  : GeomPrim(xval, yval, zval)
{
  mElements(4,1) = 0.0;

  mChanged = true;
  mLength = length();
}


/**Constructor.
 * @param m a Matrix
 */
Vector::Vector(const Matrix& m)
  : GeomPrim(m)
{
  if (mElements(4,1) != 0)
    {
      cout << "Can't construct a Vector from the given Matrix object since the 4th element is not equal to 0.0." << endl;
      Exception e("Vector::ctor(...)");
      throw e;
    }

  mChanged = true;
  mLength = length();
}


/**Constructor.
 * @param g a GeomPrim
 */
Vector::Vector(const GeomPrim& g)
  : GeomPrim(g)
{
  if (mElements(4,1) != 0)
    {
      cout << "Can't construct a Vector from the given Matrix object since the 4th element is not equal to 0.0." << endl;
      Exception e("Vector::ctor(...)");
      throw e;
    }

  mChanged = true;
  mLength = length();
}

/**Destructor */
Vector::~Vector()
{
}


/**Calculates the length of the vector
 * @return the length
 */
float
Vector::length()
{
  if (mChanged)
    {
      float t;
      t = x() * x();
      t += y() * y();
      t += z() * z();

      return sqrt(t);
    }

  return mLength;
}


/**RO-getter for x.
 * @return the value
 */
float
Vector::x() const
{
  return GeomPrim::_x();
}


/**RW-getter for x.
 * @return a reference to the x-element
 */
float&
Vector::x()
{
  return GeomPrim::_x();
}


/**Setter function for x.
 * @param x the new x value
 */
void
Vector::x(float x)
{
  GeomPrim::_x(x);
}


/**RO-getter for y.
 * @return the value
 */
float
Vector::y() const
{
  return GeomPrim::_y();
}


/**RW-getter for y.
 * @return a reference to the y-element
 */
float&
Vector::y()
{
  return GeomPrim::_y();
}


/**Setter function for y.
 * @param y the new y value
 */
void
Vector::y(float y)
{
  GeomPrim::_y(y);
}


/**RO-getter for z.
 * @return the value
 */
float
Vector::z() const
{
  return GeomPrim::_z();
}


/**RW-getter for z.
 * @return a reference to the z-element
 */
float&
Vector::z()
{
  return GeomPrim::_z();
}


/**Setter function for z.
 * @param z the new z value
 */
void
Vector::z(float z)
{
  GeomPrim::_z(z);
}


/**Apply a transform to the vector.
 * @param t a reference to the transform
 */
void
Vector::apply_transform(const Transform& t)
{
  _apply_transform(t);
  mChanged = true;
}


/**Convenience function to rotate the vector around the x-axis.
 * @param angle the angle
 */
void
Vector::rotate_x(float angle)
{
  _rotate_x(angle);
}


/**Convenience function to rotate the vector around the y-axis.
 * @param angle the angle
 */
void
Vector::rotate_y(float angle)
{
  _rotate_y(angle);
}


/**Convenience function to rotate the vector around the z-axis.
 * @param angle the angle
 */
void
Vector::rotate_z(float angle)
{
  _rotate_z(angle);
}


/**Convenience function to scale the vector along the axes of the
 * coordinate system.
 * @param scale_x scale factor along the x-axis
 * @param scale_y scale factor along the y-axis
 * @param scale_z scale factor along the z-axis
 */
void
Vector::scale(float scale_x, float scale_y, float scale_z)
{
  x() *= scale_x;
  y() *= scale_y;
  z() *= scale_z;
}


/**Convenience function to scale the length of the vector.
 * @param l the scaling factor
 */
void
Vector::scale_length(float l)
{
  scale(l, l, l);
}


/**Brings the vector to unit-length.
 * @return a reference to itself
 */
Vector&
Vector::unit()
{
  float len = length();

  x() /= len;
  y() /= len;
  z() /= len;

  return *this;
}


/**Assignment operator.
 * @param v the rhs vector
 * @return a reference of the lhs vector (this)
 */
Vector&
Vector::operator=(const Vector& v)
{
  GeomPrim::operator=(v);
  mLength = v.mLength;
  mChanged = v.mChanged;

  return *this;
}


/**Addition operator.
 * Adds two vectors.
 * @param v the rhs vector
 * @return the resulting vector
 */
Vector
Vector::operator+(const Vector& v) const
{
  Vector result;

  result.x() = x() + v.x();
  result.y() = y() + v.y();
  result.z() = z() + v.z();

  return result;
}


/**Add-assign operator.
 * @param v the lhs vector
 * @return a reference to the resulting vector (this)
 */
Vector&
Vector::operator+=(const Vector& v)
{
  *this = *this + v;

  return *this;
}


/** Substraction operator.
 * @param v the lhs vector
 * @return the resulting vector
 */
Vector
Vector::operator-(const Vector& v) const
{
  Vector result;

  result.x() = x() - v.x();
  result.y() = y() - v.y();
  result.z() = z() - v.z();

  return result;
}


/** Sub-assign operator.
 * @param v the lhs vector
 * @return a reference to the resulting vector (this)
 */
Vector&
Vector::operator-=(const Vector& v)
{
  *this = *this - v;

  return *this;
}


/**Scalar multiplikation.
 * @param f the scalar
 * @return the resulting vector
 */
Vector
Vector::operator*(const float& f) const
{
  Vector result;

  result.x() = x() * f;
  result.y() = y() * f;
  result.z() = z() * f;

  return result;
}


/**In-place scalar multiplikation.
 * @param f the scalar
 * @return a reference to thre resulting vector (this)
 */
Vector&
Vector::operator*=(const float& f)
{
  *this = *this * f;

  return *this;
}


/**Cross-product.
 * @param v the lhs vector
 * @return the cross-product
 */
Vector
Vector::operator%(const Vector& v) const
{
  Vector result;

  result.x() = y() * v.z() - z() * v.y(); 
  result.y() = z() * v.x() - x() * v.z();
  result.z() = x() * v.y() - y() * v.x();

  return result;
}


/**In-place cross-product.
 * @param v the lhs vector
 * @return a reference to the resulting vector (this=
 */
Vector&
Vector::operator%=(const Vector& v)
{
  *this = *this % v;

  return *this;
}
