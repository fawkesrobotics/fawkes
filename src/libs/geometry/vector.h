
/***************************************************************************
 *  vector.h - (Homogenous) vector
 *
 *  Created: Wed Sep 26 16:58:51 2007
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

#ifndef __VECTOR_H_
#define __VECTOR_H_

#include <geometry/geom_prim.h>
#include <iostream>

class Vector : public GeomPrim
{
 public:
  Vector(float x = 0.0, float y = 0.0, float z = 0.0);
  Vector(const Matrix& m);
  Vector(const GeomPrim& g);
  virtual ~Vector();

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

  void apply_transform(const Transform& t);

  void rotate_x(float angle);
  void rotate_y(float angle);
  void rotate_z(float angle);
  void scale(float x, float y, float z);
  void scale_length(float l);
  Vector& unit();

  Vector& operator=(const Vector& v);
  Vector operator+(const Vector& v) const;
  Vector& operator+=(const Vector& v);
  Vector operator-(const Vector& v) const;
  Vector& operator-=(const Vector& v);
  Vector operator*(const float& f) const;
  Vector& operator*=(const float& f);
  Vector operator%(const Vector& v) const;
  Vector& operator%=(const Vector& v);

 private:
  float mLength;
  bool mChanged;
};

#endif /* __VECTOR_H_ */
