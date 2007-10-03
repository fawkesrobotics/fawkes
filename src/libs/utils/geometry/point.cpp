
/***************************************************************************
 *  point.cpp - (Homogenous) point
 *
 *  Created: Thu Sep 27 17:01:55 2007
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

#include <utils/geometry/point.h>
#include <utils/geometry/vector.h>
#include <core/exception.h>

using namespace std;


/** @class Point libs/utils/geometry/point.h
 * A (homogeneous) point.
 */


/**Constructor.
 * @param xval the x-coordinate
 * @param yval the y-coordinate
 * @param zval the z-coordinate
 */
Point::Point(float xval, float yval, float zval)
  : GeomPrim(xval, yval, zval)
{
  mElements(4,1) = 1.0;
}


/**Constructor.
 * @param m a Matrix
 */
Point::Point(const Matrix& m)
  : GeomPrim(m)
{
  if (mElements(4,1) != 1.0)
    {
      cout << "Can't construct a Point from the given Matrix object since the 4th element is not equal to 1.0." << endl;
      Exception e("Point::ctor(...)");
      throw e;
    }
}


/**Constructor.
 * @param g a GeomPrim
 */
Point::Point(const GeomPrim& g)
  : GeomPrim(g)
{
  if (mElements(4,1) != 1)
    {
      cout << "Can't construct a Point from the given GeomPrim object since the 4th element is not equal to 1.0." << endl;
      Exception e("Point::ctor(...)");
      throw e;
    }
}

/**Destructor */
Point::~Point()
{
}


/**RO-getter for x.
 * @return the value
 */
float
Point::x() const
{
  return _x();
}


/**RW-getter for x.
 * @return a reference to the x-element
 */
float&
Point::x()
{
  return _x();
}


/**Setter function for x.
 * @param x the new x value
 */
void
Point::x(float x)
{
  _x(x);
}


/**RO-getter for y.
 * @return the value
 */
float
Point::y() const
{
  return _y();
}


/**RW-getter for y.
 * @return a reference to the y-element
 */
float&
Point::y()
{
  return _y();
}


/**Setter function for y.
 * @param y the new y value
 */
void
Point::y(float y)
{
  _y(y);
}


/**RO-getter for z.
 * @return the value
 */
float
Point::z() const
{
  return _z();
}


/**RW-getter for z.
 * @return a reference to the z-element
 */
float&
Point::z()
{
  return _z();
}


/**Setter function for z.
 * @param z the new z value
 */
void
Point::z(float z)
{
  _z(z);
}


/**Apply a transform to the vector.
 * @param t a reference to the transform
 */
void
Point::apply_transform(const Transform& t)
{
  _apply_transform(t);
}


/**Convenience function to rotate the point around the x-axis.
 * @param angle the angle
 */
void
Point::rotate_x(float angle)
{
  _rotate_x(angle);
}


/**Convenience function to rotate the point around the y-axis.
 * @param angle the angle
 */
void
Point::rotate_y(float angle)
{
  _rotate_y(angle);
}


/**Convenience function to rotate the point around the z-axis.
 * @param angle the angle
 */
void
Point::rotate_z(float angle)
{
  _rotate_z(angle);
}


/**Convenience function to translate the point.
 * @param trans_x translation along the x-axis
 * @param trans_y translation along the y-axis
 * @param trans_z translation along the z-axis
 */
void
Point::trans(float trans_x, float trans_y, float trans_z)
{
  x() += trans_x;
  y() += trans_y;
  z() += trans_z;
}


/**Moves the point to the specified location.
 * @param xval the new x-coordinate
 * @param yval the new y-coordinate
 * @param zval the new z-coordinate
 */
void
Point::move_to(float xval, float yval, float zval)
{
  x() = xval;
  y() = yval;
  z() = zval;
}


/**Assignment operator.
 * @param p the rhs point
 * @return a reference of the lhs point (this)
 */
Point&
Point::operator=(const Point& p)
{
  GeomPrim::operator=(p);

  return *this;
}


/**Adds a vector to a point.
 * @param v the rhs vector
 * @return the resulting point
 */
Point
Point::operator+(const Vector& v)
{
  Point result;

  result.x() = x() + v.x();
  result.y() = y() + v.y();
  result.z() = z() + v.z();

  return result;
}


/**Add-assign operator.
 * @param v the rhs vector
 * @return a reference to the resulting point (this)
 */
Point&
Point::operator+=(const Vector& v)
{
  *this = *this + v;

  return *this;
}


/**The difference between two points is a vector.
 * @param p the rhs point
 * @return the difference vector
 */
Vector
Point::operator-(const Point& p) const
{
  Vector v1(mElements);
  Vector v2(p.mElements);
  
  return v1 - v2;
}
