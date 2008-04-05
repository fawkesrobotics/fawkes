
/***************************************************************************
 *  transform.h - (Homogenous) affine transformation
 *
 *  Created: Wed Sep 26 14:31:42 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __TRANSFORM_H_
#define __TRANSFORM_H_

#include <geometry/matrix.h>

class Vector;
class GeomPrim;

class HomTransform
{
  friend std::ostream& operator<<(std::ostream& ostr, const HomTransform& t);

 public:
  HomTransform();
  HomTransform( float alpha,
		float beta,
		float gamma,
		float trans_x = 0.0,
		float trans_y = 0.0,
		float trans_z = 0.0 );

  HomTransform(const Matrix& m);

  virtual ~HomTransform();

  Matrix get_homtransmat() const;
  Matrix get_rotmat() const;
  Matrix get_trans() const;

  HomTransform& id();

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

  HomTransform& inverse();
  HomTransform get_inverse() const;

  HomTransform& operator=(const HomTransform& t);
  HomTransform operator*(const HomTransform& t) const;
  HomTransform& operator*=(const HomTransform& t);
  GeomPrim operator*(const GeomPrim& g) const;
  bool operator==(const HomTransform& t) const;

  void print_to_stream(std::ostream& ostr) const;

 private:
  void rotmat2euler( const Matrix& m,
		     float& alpha, float& beta, float& gamma ) const;

  Matrix mMatrix;
};

#endif /* __TRANSFORM_H_ */
