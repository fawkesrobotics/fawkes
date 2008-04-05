
/***************************************************************************
 *  vector.h - (Homogenous) vector
 *
 *  Created: Wed Sep 26 16:58:51 2007
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

#ifndef __VECTOR_H_
#define __VECTOR_H_

#include <geometry/geom_prim.h>
#include <iostream>

class HomVector : public GeomPrim
{
 public:
  HomVector(float x = 0.0, float y = 0.0, float z = 0.0);
  HomVector(const Matrix& m);
  HomVector(const GeomPrim& g);
  virtual ~HomVector();

  float length();

  float  x() const;
  float& x();
  void   x(float x);

  float  y() const;
  float& y();
  void   y(float y);

  float  z() const;
  float& z();
  void   z(float z);

  void apply_transform(const HomTransform& t);

  void rotate_x(float angle);
  void rotate_y(float angle);
  void rotate_z(float angle);
  void scale(float x, float y, float z);
  void scale_length(float l);
  HomVector& unit();

  HomVector& operator=(const HomVector& v);
  HomVector operator+(const HomVector& v) const;
  HomVector& operator+=(const HomVector& v);
  HomVector operator-(const HomVector& v) const;
  HomVector& operator-=(const HomVector& v);
  HomVector operator*(const float& f) const;
  HomVector& operator*=(const float& f);
  float operator*(const HomVector& v) const;
  HomVector operator%(const HomVector& v) const;
  HomVector& operator%=(const HomVector& v);
  float operator<(HomVector& v);

 private:
  float mLength;
  bool mChanged;
};

#endif /* __VECTOR_H_ */
