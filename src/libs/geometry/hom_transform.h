
/***************************************************************************
 *  hom_transform.h - Homogenous affine transformation
 *
 *  Created: Wed Sep 26 14:31:42 2007
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

#ifndef __GEOMETRY_TRANSFORM_H_
#define __GEOMETRY_TRANSFORM_H_

class Matrix;
class HomCoord;
class HomVector;
class HomPoint;

class HomTransform
{
 public:
  HomTransform();
  HomTransform(const Matrix& m);
  ~HomTransform();

  HomTransform& reset();
  HomTransform& invert();
  HomTransform  get_inverse();
  
  void rotate_x(float rad);
  void rotate_y(float rad);
  void rotate_z(float rad);
  
  void trans(float dx, float dy, float dz);

  HomTransform& operator=(const HomTransform& t);

  HomTransform  operator*(const HomTransform& t) const;
  HomTransform& operator*=(const HomTransform& t);

  bool operator==(const HomTransform& t) const;

  HomCoord  operator*(const HomCoord& h) const;

 private:
  Matrix* m_matrix;
};

#endif /* __GEOMETRY_TRANSFORM_H_ */
