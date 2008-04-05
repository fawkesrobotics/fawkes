
/***************************************************************************
 *  point.h - (Homogenous) point
 *
 *  Created: Thu Sep 27 16:55:50 2007
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

#ifndef __POINT_H_
#define __POINT_H_

#include <geometry/geom_prim.h>

class HomVector;

class HomPoint : public GeomPrim
{
 public:
  HomPoint(float x = 0.0, float y = 0.0, float z = 0.0);
  HomPoint(const Matrix& m);
  HomPoint(const GeomPrim& g);
  virtual ~HomPoint();

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

  void trans(float trans_x, float trans_y, float trans_z);
  void move_to(float x, float y, float z);

  HomPoint& operator=(const HomPoint& p);
  HomPoint operator+(const HomVector& v);
  HomPoint& operator+=(const HomVector& p);
  HomVector operator-(const HomPoint& p) const;
};

#endif /* __POINT_H_ */
