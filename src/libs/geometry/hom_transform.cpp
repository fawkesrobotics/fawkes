
/***************************************************************************
 *  hom_transform.h - Homogenous affine transformation
 *
 *  Created: Wed Sep 26 14:47:35 2007
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

#include "hom_transform.h"
#include "hom_coord.h"
#include "matrix.h"

#include <core/exceptions/software.h>

#include <cmath>

namespace fawkes {

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
 * @param t a HomTransform
 */
HomTransform::HomTransform(const HomTransform& t)
{
  m_matrix = new Matrix(*(t.m_matrix));
}

/** Constructor from a Matrix.
 * @param m a Matrix
 */
HomTransform::HomTransform(const Matrix& m)
{
  if ((m.num_rows() != 4) || (m.num_cols() != 4))
  {
    throw fawkes::IllegalArgumentException("The matrix to create a HomTransform has to be 4x4.");
  }

  m_matrix = new Matrix(m);
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
  float ct[3] = { (*m_matrix)(0, 3), (*m_matrix)(1, 3), (*m_matrix)(2, 3) };
  Matrix rot  = m_matrix->get_submatrix(0, 0, 3, 3);

  m_matrix->overlay(0, 0, rot.transpose());
  (*m_matrix)(0, 3) = -ct[0] * (*m_matrix)(0, 0) - ct[1] * (*m_matrix)(0, 1) - ct[2] * (*m_matrix)(0, 2);
  (*m_matrix)(1, 3) = -ct[0] * (*m_matrix)(1, 0) - ct[1] * (*m_matrix)(1, 1) - ct[2] * (*m_matrix)(1, 2);
  (*m_matrix)(2, 3) = -ct[0] * (*m_matrix)(2, 0) - ct[1] * (*m_matrix)(2, 1) - ct[2] * (*m_matrix)(2, 2);

  return *this;
}

/** Obtain inverse transform.
 * @return the inverse transform
 */
HomTransform
HomTransform::get_inverse()
{
  HomTransform t(*this);
  t.invert();

  return t;
}

/** Add rotation around the x-axis.
 * @param rad rotation angle in rad
 */
void
HomTransform::rotate_x(float rad)
{
  float cos = cosf(rad);
  float sin = sinf(rad);
  float s1[3] = { (*m_matrix)(0,1), (*m_matrix)(1,1), (*m_matrix)(2,1) };
  float s2[3] = { (*m_matrix)(0,2), (*m_matrix)(1,2), (*m_matrix)(2,2) };

  (*m_matrix)(0,1) = s1[0] * cos + s2[0] * sin;
  (*m_matrix)(1,1) = s1[1] * cos + s2[1] * sin;
  (*m_matrix)(2,1) = s1[2] * cos + s2[2] * sin;
  (*m_matrix)(0,2) = -s1[0] * sin + s2[0] * cos;
  (*m_matrix)(1,2) = -s1[1] * sin + s2[1] * cos;
  (*m_matrix)(2,2) = -s1[2] * sin + s2[2] * cos;
}

/** Add rotation around the y-axis.
 * @param rad rotation angle in rad
 */
void
HomTransform::rotate_y(float rad)
{
  float cos = cosf(rad);
  float sin = sinf(rad);
  float s1[3] = { (*m_matrix)(0,0), (*m_matrix)(1,0), (*m_matrix)(2,0) };
  float s2[3] = { (*m_matrix)(0,2), (*m_matrix)(1,2), (*m_matrix)(2,2) };

  (*m_matrix)(0,0) = s1[0] * cos - s2[0] * sin;
  (*m_matrix)(1,0) = s1[1] * cos - s2[1] * sin;
  (*m_matrix)(2,0) = s1[2] * cos - s2[2] * sin;

  (*m_matrix)(0,2) = s1[0] * sin + s2[0] * cos;
  (*m_matrix)(1,2) = s1[1] * sin + s2[1] * cos;
  (*m_matrix)(2,2) = s1[2] * sin + s2[2] * cos;
}

/** Add rotation around the z-axis.
 * @param rad rotation angle in rad
 */
void
HomTransform::rotate_z(float rad)
{
  float cos = cosf(rad);
  float sin = sinf(rad);
  float s1[3] = { (*m_matrix)(0,0), (*m_matrix)(1,0), (*m_matrix)(2,0) };
  float s2[3] = { (*m_matrix)(0,1), (*m_matrix)(1,1), (*m_matrix)(2,1) };

  (*m_matrix)(0,0) = s1[0] * cos + s2[0] * sin;
  (*m_matrix)(1,0) = s1[1] * cos + s2[1] * sin;
  (*m_matrix)(2,0) = s1[2] * cos + s2[2] * sin;

  (*m_matrix)(0,1) = -s1[0] * sin + s2[0] * cos;
  (*m_matrix)(1,1) = -s1[1] * sin + s2[1] * cos;
  (*m_matrix)(2,1) = -s1[2] * sin + s2[2] * cos;
}

/** Add translation to the transformation.
 * @param dx offset along x-axis
 * @param dy offset along y-axis
 * @param dz offset along z-axis
 */
void
HomTransform::trans(float dx, float dy, float dz)
{
  (*m_matrix)(0, 3) += (*m_matrix)(0, 0) * dx + (*m_matrix)(0, 1) * dy + (*m_matrix)(0, 2) * dz;
  (*m_matrix)(1, 3) += (*m_matrix)(1, 0) * dx + (*m_matrix)(1, 1) * dy + (*m_matrix)(1, 2) * dz;
  (*m_matrix)(2, 3) += (*m_matrix)(2, 0) * dx + (*m_matrix)(2, 1) * dy + (*m_matrix)(2, 2) * dz;
}


/** Modified Denavit-Hartenberg transformation.
 * DH-transformation as used by Aldebaran
 * @see http://robocup.aldebaran-robotics.com/index.php?option=com_content&task=view&id=30#id2514205 "3.2.2.1.3.2. Forward kinematics model parameters"
 *
 * @param alpha the angle from the Z_i-1 axis to the Z_i axis about the X_i-1 axis
 * @param a     the offset distance between the Z_i-1 and Z_i axes along the X_i-1 axis
 * @param theta the angle between the X_i-1 and X_i axes about the Z_i axis
 * @param d     the distance from the origin of frame X_i-1 to the X_i axis along the Z_i axis
 */
void
HomTransform::mDH(const float alpha, const float a, const float theta, const float d)
{
  if (alpha)  rotate_x(alpha);
  if (a || d) trans(a, 0, d);
  if (theta)  rotate_z(theta);
}


/** Set the translation.
 * @param x the translation along the x-axis
 * @param y the translation along the y-axis
 * @param z the translation along the z-axis
 */
void
HomTransform::set_trans(float x, float y, float z)
{
  Matrix& matrix_ref = *m_matrix;
  matrix_ref(0, 3) = x;
  matrix_ref(1, 3) = y;
  matrix_ref(2, 3) = z;
}

/** Assignment operator.
 * @param t the other transformation
 * @return reference to the lhs transformation
 */
HomTransform&
HomTransform::operator=(const HomTransform& t)
{
  (*m_matrix) = *(t.m_matrix);

  return *this;
}

/** Multiplication-assignment operator.
 * @param t the rhs transformation
 * @return reference to the result of the multiplication
 */
HomTransform&
HomTransform::operator*=(const HomTransform& t)
{
  (*m_matrix) *= (*t.m_matrix);

  return *this;
}

/** Comparison operator.
 * @param t the other transformation
 * @return true, if both transormations are equal
 */
bool
HomTransform::operator==(const HomTransform& t) const
{
  return ((*m_matrix) == *(t.m_matrix));
}

/** Prints the matrix.
 * @param name Heading of the output
 * @param col_sep a string used to separate columns (defaults to '\\t')
 * @param row_sep a string used to separate rows (defaults to '\\n')
 */
void
HomTransform::print_info(const char *name, const char *col_sep, const char *row_sep) const
{
  m_matrix->print_info(name ? name : "HomTransform", col_sep, row_sep);
}


/** Returns a copy of the matrix.
 * @return the matrix of the transformation
 */
const Matrix&
HomTransform::get_matrix() const
{
  return *m_matrix;
}

} // end namespace fawkes

