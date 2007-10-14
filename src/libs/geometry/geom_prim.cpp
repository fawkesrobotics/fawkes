
/***************************************************************************
 *  geom_prim.cpp - Geometric Primitive
 *
 *  Created: Thu Sep 27 16:21:24 2007
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

#include <geometry/geom_prim.h>
#include <geometry/transform.h>
#include <core/exception.h>

using namespace std;


/** @class GeomPrim libs/geometry/geom_prim.h
 * Base class for geometric primitives (Vector and Point).
 */

/** @var GeomPrim::mElements
 * A single-column matrix containing the elements of the vector.
 */


/**Constructor.
 * @param xval the x-coordinate
 * @param yval the y-coordinate
 * @param zval the z-coordinate
 */
GeomPrim::GeomPrim(float xval, float yval, float zval)
  : mElements(4, 1)
{
  _x(xval);
  _y(yval);
  _z(zval);
}


/**Constructor.
 * @param m a Matrix
 */
GeomPrim::GeomPrim(const Matrix& m)
  : mElements(m)
{
  unsigned  int rows, cols;
  mElements.get_dimensions(&rows, &cols);
  
  if ( rows != 4 || cols != 1 )
    {
      cout << "GeomPrim can't be contstructed from a " << rows << " x " << cols <<" matrix" << endl;
      Exception e("GeomPrim ctor");
      throw e;
    }
}


/**Destructor */
GeomPrim::~GeomPrim()
{
}


/**RO-getter for x.
 * @return the value
 */
float
GeomPrim::_x() const
{
  return mElements(1,1);
}


/**RW-getter for x.
 * @return a reference to the x-element
 */
float&
GeomPrim::_x()
{
  return mElements(1,1);
}


/**Setter function for x.
 * @param x the new x value
 */
void
GeomPrim::_x(float x)
{
  mElements(1,1) = x;
}


/**RO-getter for y.
 * @return the value
 */
float
GeomPrim::_y() const
{
  return mElements(2,1);
}


/**RW-getter for y.
 * @return a reference to the y-element
 */
float&
GeomPrim::_y()
{
  return mElements(2,1);
}


/**Setter function for y.
 * @param y the new y value
 */
void
GeomPrim::_y(float y)
{
  mElements(2,1) = y;
}


/**RO-getter for z.
 * @return the value
 */
float
GeomPrim::_z() const
{
  return mElements(3,1);
}


/**RW-getter for z.
 * @return a reference to the z-element
 */
float&
GeomPrim::_z()
{
  return mElements(3,1);
}


/**Setter function for z.
 * @param z the new z value
 */
void
GeomPrim::_z(float z)
{
  mElements(3,1) = z;
}


/**Apply a transform to the vector.
 * @param t a reference to the transform
 */
void
GeomPrim::_apply_transform(const Transform& t)
{
  mElements = t.get_homtransmat() * mElements;
}


/**Convenience function to rotate the vector around the x-axis.
 * @param angle the angle
 */
void
GeomPrim::_rotate_x(float angle)
{
  Transform t;
  t.rotate_x(angle);
  _apply_transform(t);
}


/**Convenience function to rotate the vector around the y-axis.
 * @param angle the angle
 */
void
GeomPrim::_rotate_y(float angle)
{
  Transform t;
  t.rotate_y(angle);
  _apply_transform(t);
}


/**Convenience function to rotate the vector around the z-axis.
 * @param angle the angle
 */
void
GeomPrim::_rotate_z(float angle)
{
  Transform t;
  t.rotate_z(angle);
  _apply_transform(t);
}


/**Assignment operator.
 * @param g the rhs GeomPrim
 * @return a reference of the lhs GeomPrim (this)
 */
GeomPrim&
GeomPrim::operator=(const GeomPrim& g)
{
  mElements = g.mElements;

  return *this;
}


/**Output function.
 * @param ostr the output stream
 */
void
GeomPrim::_print_to_stream(std::ostream& ostr) const
{
  ostr << "[";
  for (unsigned int row = 1; row <= 4; row++)
    {
      ostr << " " << mElements(row,1);
    }
  ostr << " ]T";
}


std::ostream& operator<<(std::ostream& ostr, const GeomPrim& g)
{
  g._print_to_stream(ostr);

  return ostr;
}
