
/***************************************************************************
 *  transform.h - (Homogenous) affine transformation
 *
 *  Created: Wed Sep 26 14:47:35 2007
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

#include <geometry/transform.h>
#include <geometry/vector.h>
#include <geometry/geom_prim.h>
#include <core/exception.h>

#include <cmath>

using namespace std;


/** @class Transform libs/geometry/transform.h
 * This class describes a homogeneous transformation.
 */

/** @var Transform::mMatrix
 * The 4x4 matrix defining the transform.
 */


/**Constructor. */
Transform::Transform() : mMatrix(4, 4)
{
  mMatrix.id();
}

/**Constructor.
 * @param alpha
 * @param beta
 * @param gamma
 * @param trans_x transformation along the x-axis (after rotation)
 * @param trans_y transformation along the y-axis (after rotation)
 * @param trans_z transformation along the z-axis (after rotation)
 */
Transform::Transform( float alpha,
		      float beta,
		      float gamma,
		      float trans_x,
		      float trans_y,
		      float trans_z )
  : mMatrix(4, 4)
{
  mMatrix.id();

  rotate_euler(alpha, beta, gamma);
  trans(trans_x, trans_y, trans_z);
}


/**Constructor 
 * @param m a Matrix from which a Transform is constructed
 */
Transform::Transform(const Matrix& m)
  : mMatrix(m)
{
  unsigned int rows, cols;
  mMatrix.get_dimensions(&rows, &cols);

  if ( rows != 4 || cols != 4 )
    {
      cout << "Can't construct a Transform from a " 
	   << rows << " x " << cols << " matrix." << endl;
      Exception e("Transform::ctor(...)");
      throw e;
    }
  else
    {
      mMatrix = m;
    }
}

/**Destructor */
Transform::~Transform()
{
}


/**Returns the matrix of the current transform.
 * @return the trans matrix
 */
Matrix
Transform::get_homtransmat() const
{
  return mMatrix;
}


/**Returns the current rotation matrix.
 * @return the rotation matrix
 */
Matrix
Transform::get_rotmat() const
{
  Matrix m = mMatrix.submatrix(1, 1, 3, 3);

  return m;
}


/**Returns the current translation.
 * @return a translation (vector)
 */
Matrix
Transform::get_trans() const
{
  Matrix m = mMatrix.submatrix(1, 4, 3, 1);

  return m;
}


/**Reset the transform.
 */
Transform&
Transform::id()
{
  mMatrix.id();

  return *this;
}


/**Adds a rotation around the x-axis.
 * @param angle the angle
 */
void
Transform::rotate_x(float angle)
{
  Matrix m(3, 3);
  m.id();
  
  m(2,2) = cos(angle);
  m(2,3) = -sin(angle);
  m(3,2) = sin(angle);
  m(3,3) = cos(angle);

  Matrix rot_mat = mMatrix.submatrix(1, 1, 3, 3);
  rot_mat *= m;
  
  mMatrix.overlay(1, 1, rot_mat);
}


/**Adds a rotation around the y-axis.
 * @param angle the angle
 */
void
Transform::rotate_y(float angle)
{
  Matrix m(3, 3);
  m.id();
  
  m(1,1) = cos(angle);
  m(1,3) = sin(angle);
  m(3,1) = -sin(angle);
  m(3,3) = cos(angle);

  Matrix rot_mat = mMatrix.submatrix(1, 1, 3, 3);
  rot_mat *= m;
  
  mMatrix.overlay(1, 1, rot_mat);
}


/**Adds a rotation around the z-axis.
 * @param angle the angle
 */
void
Transform::rotate_z(float angle)
{
  Matrix m(3, 3);
  m.id();
  
  m(1,1) = cos(angle);
  m(1,2) = -sin(angle);
  m(2,1) = sin(angle);
  m(2,2) = cos(angle);

  Matrix rot_mat = mMatrix.submatrix(1, 1, 3, 3);
  rot_mat *= m;
  
  mMatrix.overlay(1, 1, rot_mat);
}


/**Adds a rotation to the transform specified as Euler angles.
 * @param alpha
 * @param beta
 * @param gamma
 */
void
Transform::rotate_euler( float alpha,
			 float beta,
			 float gamma )
{
  Transform t_alpha, t_beta, t_gamma;

  t_alpha.rotate_z(alpha);
  t_beta.rotate_y(beta);
  t_gamma.rotate_z(gamma);

  *this *= t_alpha * t_beta * t_gamma;
}


/**Adds a rotation to the transform specified as Roll-Pitch-Yaw.
 * @param roll
 * @param pitch
 * @param yaw
 */
void
Transform::rotate_rpy( float roll,
		       float pitch,
		       float yaw )
{
  Transform t_roll, t_pitch, t_yaw;

  t_roll.rotate_z(roll);
  t_pitch.rotate_y(pitch);
  t_yaw.rotate_x(yaw);

  *this *= t_roll * t_pitch * t_yaw;
}


/**Adds a translation to the transform.
 * @param trans_x translation along the x-axis
 * @param trans_y translation along the z-axis
 * @param trans_z translation along the y-axis
 */
void
Transform::trans(float trans_x,
		 float trans_y,
		 float trans_z)
{
  mMatrix(1,4) += trans_x;
  mMatrix(2,4) += trans_y;
  mMatrix(3,4) += trans_z;
}


/**Inverses the transform.
 * @return a reference to itself
 */
Transform&
Transform::inverse()
{
  Matrix rot = get_rotmat();
  rot.transpose();

  mMatrix.overlay(1, 1, rot);

  mMatrix(1,4) *= -1.0;
  mMatrix(2,4) *= -1.0;
  mMatrix(3,4) *= -1.0;

  return *this;
}


/**Returns the inverse of the transform
 * @return the inversed transform
 */
Transform
Transform::get_inverse() const
{
  Transform t = *this;

  t.inverse();

  return t;
}


/**Assignment operator.
 * @param t the rhs operand
 * @return a reference to the result (this) 
 */
Transform&
Transform::operator=(const Transform& t)
{
  mMatrix = t.mMatrix;

  return *this;
}


/**Adds one transform to another one.
 * @param t the other transform
 * @return the resulting transform
 */
Transform
Transform::operator*(const Transform& t) const
{
  Transform result;

  result.mMatrix = mMatrix * t.mMatrix;

  return result;
}


/**Adds two transforms and assigns the result to the lhs operand.
 * @param t the other transform
 * @return a reference to the result (this)
 */
Transform&
Transform::operator*=(const Transform& t)
{
  *this = *this * t;

  return *this;
}


/**Apply a transform to a GeomPrim.
 * @param g the GeomPrim
 * @return the transfromed GeomPrim
 */
GeomPrim
Transform::operator*(const GeomPrim& g) const
{
  GeomPrim result = g;

  result._apply_transform(*this);

  return result;
}


/**Comparison operator.
 * @param t the other transform
 * @return true if the transforms are euqal
 */
bool
Transform::operator==(const Transform& t) const
{
  if (mMatrix == t.mMatrix)
    {
      return true;
    }

  return false;
}


/**Extracts from a rotation matrix the Euler angles.
 * @param m a reference to the rotation matrix
 * @param alpha
 * @param beta
 * @param gamma
 */
void
Transform::rotmat2euler( const Matrix& m,
			 float& alpha,
			 float& beta,
			 float& gamma) const
{
  // TODO
}


/**Prints the transform to a stream.
 * @param ostr the output stream
 */
void
Transform::print_to_stream(std::ostream& ostr) const
{
//   float alpha, beta, gamma;
//   rotmat2euler(mMatrix, alpha, beta, gamma);

//   float trans_x = mMatrix(1,4);
//   float trans_y = mMatrix(2,4);
//   float trans_z = mMatrix(3,4);

//   ostr << "Rotation: (" << alpha << ", " << beta << ", " << gamma << " )" << std::endl;
//   ostr << "Translation: (" << trans_x << ", " << trans_y << ", " << trans_z << " )" << std::endl;
  ostr << mMatrix;
}


std::ostream& operator<<(std::ostream& ostr, const Transform& t)
{
  t.print_to_stream(ostr);

  return ostr;
}
