
/***************************************************************************
 *  transform.h - (Homogenous) affine transformation
 *
 *  Created: Wed Sep 26 14:31:42 2007
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

#ifndef __TRANSFORM_H_
#define __TRANSFORM_H_

#include <geometry/matrix.h>

class Vector;
class GeomPrim;

class Transform
{
  friend std::ostream& operator<<(std::ostream& ostr, const Transform& t);

 public:
  Transform();
  Transform( float alpha,
	     float beta,
	     float gamma,
	     float trans_x = 0.0,
	     float trans_y = 0.0,
	     float trans_z = 0.0 );

  Transform(const Matrix& m);

  virtual ~Transform();

  Matrix get_homtransmat() const;
  Matrix get_rotmat() const;
  Matrix get_trans() const;

  Transform& id();

  void rotate_x(float angle);
  void rotate_y(float angle);
  void rotate_z(float angle);

  void rotate_euler( float alpha,
		     float beta,
		     float gamma );

  void rotate_rpy( float roll,
		   float pitch,
		   float yaw );

  void trans( float tran_x,
	      float trans_y,
	      float trans_z );

  Transform& inverse();
  Transform get_inverse() const;

  Transform& operator=(const Transform& t);
  Transform operator*(const Transform& t) const;
  Transform& operator*=(const Transform& t);
  GeomPrim operator*(const GeomPrim& g) const;
  bool operator==(const Transform& t) const;

  void print_to_stream(std::ostream& ostr) const;

 private:
  void rotmat2euler( const Matrix& m,
		     float& alpha, float& beta, float& gamma ) const;

  Matrix mMatrix;
};

#endif /* __TRANSFORM_H_ */
