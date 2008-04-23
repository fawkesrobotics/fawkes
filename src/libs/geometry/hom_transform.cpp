
/***************************************************************************
 *  hom_transform.h - Homogenous affine transformation
 *
 *  Created: Wed Sep 26 14:47:35 2007
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

#include <geometry/hom_transform.h>
#include <geometry/hom_coord.h>
#include <geometry/hom_vector.h>
#include <geometry/hom_point.h>
#include <geometry/matrix.h>
#include <geometry/vector.h>

#include <cmath>

/** @class HomTransform geometry/hom_transform.h
 * This class describes a homogeneous transformation.
 * @author Daniel Beck
 */

/** Constructor. */
HomTransform::HomTransform()
{
  m_matrix = new Matrix(4, 4);
  m_matrix->id();
}

/** Copy constructor.
 * @param m a Matrix
 */
HomTransform::HomTransform(const Matrix& m)
{
  m_matrix = new Matrix(4, 4);
  (*m_matrix) = m;
}

/** Destructor */
HomTransform::~HomTransform()
{
  delete m_matrix;
}

/** Reset transformation. */
HomTransform&
HomTransform::reset()
{
  m_matrix->id();
  return *this;
}

/** Invert the transformation.
 * @return reference to the inverted transformation
 */
HomTransform&
HomTransform::invert()
{
  Matrix rot   = m_matrix->get_submatrix(0, 0, 3, 3);

  Matrix inv(4, 4);
  inv.id();
  inv.overlay( 0, 0, rot.get_inverse() );
  inv(0, 3) = -(*m_matrix)(0, 3);
  inv(1, 3) = -(*m_matrix)(1, 3);
  inv(2, 3) = -(*m_matrix)(2, 3);
  
  // TODO: proj. part

  return *this;
}

/** Obtain inverse transformatoin.
 * @return the invers transformation
 */ 
HomTransform
HomTransform::get_inverse()
{
  HomTransform t = *this;
  t.invert();

  return t;
}
  
/** Add rotation around the x-axis.
 * @param rad rotation angle in rad
 */
void
HomTransform::rotate_x(float rad)
{
  Matrix m(3, 3);
  m.id();
  
  m(1,1) =  cosf(rad);
  m(1,2) = -sinf(rad);
  m(2,1) =  sinf(rad);
  m(2,2) =  cosf(rad);

  Matrix rot = m_matrix->get_submatrix(0, 0, 3, 3);
  rot *= m;
  
  m_matrix->overlay(0, 0, rot);

}

/** Add rotation around the y-axis.
 * @param rad rotation angle in rad
 */
void
HomTransform::rotate_y(float rad)
{
  Matrix m(3, 3);
  m.id();
  
  m(0,0) =  cosf(rad);
  m(0,2) =  sinf(rad);
  m(2,0) = -sinf(rad);
  m(2,2) =  cosf(rad);

  Matrix rot = m_matrix->get_submatrix(0, 0, 3, 3);
  rot *= m;
  
  m_matrix->overlay(0, 0, rot);
}

/** Add rotation around the z-axis.
 * @param rad rotation angle in rad
 */
void
HomTransform::rotate_z(float rad)
{
  Matrix m(3, 3);
  m.id();
  
  m(0,0) =  cosf(rad);
  m(0,1) = -sinf(rad);
  m(1,0) =  sinf(rad);
  m(1,1) =  cosf(rad);

  Matrix rot = m_matrix->get_submatrix(0, 0, 3, 3);
  rot *= m;
  
  m_matrix->overlay(0, 0, rot);
}
  
/** Add translation to the transformation.
 * @param dx offset along x-axis
 * @param dy offset along y-axis
 * @param dz offset along z-axis
 */
void
HomTransform::trans(float dx, float dy, float dz)
{
  (*m_matrix)(0, 3) += dx;
  (*m_matrix)(1, 3) += dy;
  (*m_matrix)(2, 3) += dz;
}

/** Assignement operator.
 * @param t the other transformation
 * @return reference to the lhs transformation
 */
HomTransform&
HomTransform::operator=(const HomTransform& t)
{
  (*m_matrix) = (*t.m_matrix);

  return *this;
}

/** Multiplication operator.
 * @param t the rhs transformation
 * @return result of the multiplication
 */
HomTransform
HomTransform::operator*(const HomTransform& t) const
{
  Matrix m(4, 4);
  m = (*m_matrix) * (*t.m_matrix);

  return HomTransform(m);
}

/** Multiplication-assignment operator.
 * @param t the rhs transformation
 * @return reference to the result of the multiplication
 */
HomTransform&
HomTransform::operator*=(const HomTransform& t)
{
  return *this;
}

/** Comparison operator.
 * @param t the other transformation
 * @return true, if both transormations are equal
 */
bool
HomTransform::operator==(const HomTransform& t) const
{
  bool result = false;
  if ( (*m_matrix) == (*t.m_matrix) )
    { result = true; }

  return true;
}

/** Transformation operator.
 * @param h a HomCoord
 * @return the transformed HomCoord
 */
HomCoord
HomTransform::operator*(const HomCoord& h) const
{
  Vector v(4);
  v = (*m_matrix) * (*h.m_vector);
  
  return HomCoord(v);
}
