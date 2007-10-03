
/***************************************************************************
 *  geom_prim.h - Geometric Primitive
 *
 *  Created: Thu Sep 27 16:07:00 2007
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

#ifndef __GEOM_PRIM_H_
#define __GEOM_PRIM_H_

#include <utils/geometry/matrix.h>
#include <iostream>

class Transform;

class GeomPrim
{
  friend class Transform;
  friend std::ostream& operator<<(std::ostream& ostr, const GeomPrim& g);

 public:
  virtual ~GeomPrim();

 protected:
  GeomPrim(float x = 0.0, float y = 0.0, float z = 0.0);
  GeomPrim(const Matrix& m);

  float  _x() const;
  float& _x();
  void   _x(float x);

  float  _y() const;
  float& _y();
  void   _y(float y);

  float  _z() const;
  float& _z();
  void   _z(float z);

  void _apply_transform(const Transform& t);

  void _rotate_x(float angle);
  void _rotate_y(float angle);
  void _rotate_z(float angle);

  GeomPrim& operator=(const GeomPrim& g);

  void _print_to_stream(std::ostream& ostr) const;

  Matrix mElements;
};

#endif /* __GEOM_PRIM_H_ */
