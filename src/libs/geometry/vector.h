
/***************************************************************************
 *  vector.h - Column Vector
 *
 *  Created: Wed April 02 14:03:32 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __GEOMETRY_VECTOR_H_
#define __GEOMETRY_VECTOR_H_

class Vector
{
 public:
  Vector(unsigned int size = 3, float* elems = 0, bool manage_memory = true);
  ~Vector();

  unsigned int size() const;
  void         set_size(unsigned int size);

  float* data_ptr() const;

  float  get(unsigned int d) const;
  float& get(unsigned int d);
  void   set(unsigned int d, float v);

  float  x() const;
  float& x();
  void   x(float x);

  float  y() const;
  float& y();
  void   y(float y);

  float  z() const;
  float& z();
  void   z(float z);

  float  operator[](unsigned int d) const;
  float& operator[](unsigned int d);

  Vector  operator*(const float& f) const;
  Vector& operator*=(const float& f);

  Vector  operator/(const float& f) const;
  Vector& operator/=(const float& f);

  Vector& operator=(const Vector& v);

  bool operator==(const Vector& v);

  void print_info(const char* name = 0) const;

 private:
  unsigned int m_size;
  float*       m_data;
  bool         m_manage_memory;
};

#endif /* __GEOMETRY_VECTOR_H_ */
