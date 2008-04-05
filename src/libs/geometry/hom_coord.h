
/***************************************************************************
 *  hom_coord.h - Homogeneous coordinates
 *
 *  Created: Thu Sep 27 16:07:00 2007
 *  Copyright  2007-2008  Daniel Beck
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

#ifndef __GEOMETRY_HOM_COORD_H_
#define __GEOMETRY_HOM_COORD_H_

class Vector;

class HomCoord
{
 public:
  friend class HomTransform;

  virtual ~HomCoord();

  virtual float  x() const;
  virtual float& x();
  virtual void   x(float x);

  virtual float  y() const;
  virtual float& y();
  virtual void   y(float y);

  virtual float  z() const;
  virtual float& z();
  virtual void   z(float z);

  virtual float  w() const;
  virtual float& w();
  virtual void   w(float w);

  HomCoord& rotate_x(float rad);
  HomCoord& rotate_y(float rad);
  HomCoord& rotate_z(float rad);

  virtual HomCoord  operator-(const HomCoord& h) const;
  virtual HomCoord& operator-=(const HomCoord& h);

  virtual HomCoord  operator+(const HomCoord& h) const;
  virtual HomCoord& operator+=(const HomCoord& h);

  virtual HomCoord& operator=(const HomCoord& h);
  
 protected:
  HomCoord(float x = 0.0, float y = 0.0, float z = 0.0, float w = 0.0);
  HomCoord(const Vector& v);

  Vector* m_vector;
};

#endif /* __GEOMETRY_HOM_COORD_H_ */
