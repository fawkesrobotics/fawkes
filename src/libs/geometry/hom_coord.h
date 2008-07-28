
/***************************************************************************
 *  hom_coord.h - Homogeneous coordinates
 *
 *  Created: Thu Sep 27 16:07:00 2007
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

#ifndef __GEOMETRY_HOM_COORD_H_
#define __GEOMETRY_HOM_COORD_H_

#include <ostream>

namespace fawkes {

class Vector;

class HomCoord
{
 public:
  friend class HomTransform;

  HomCoord(const HomCoord& c);

  virtual ~HomCoord();

  virtual float  x() const;
  virtual float& x();
  virtual HomCoord& x(float x);

  virtual float  y() const;
  virtual float& y();
  virtual HomCoord& y(float y);

  virtual float  z() const;
  virtual float& z();
  virtual HomCoord& z(float z);

  virtual float  w() const;
  virtual float& w();
  virtual HomCoord& w(float w);

  virtual HomCoord& rotate_x(float rad);
  virtual HomCoord& rotate_y(float rad);
  virtual HomCoord& rotate_z(float rad);

  virtual HomCoord  operator-(const HomCoord& h) const;
  virtual HomCoord& operator-=(const HomCoord& h);

  virtual HomCoord  operator+(const HomCoord& h) const;
  virtual HomCoord& operator+=(const HomCoord& h);
		
	virtual float operator*(const HomCoord& h) const;
		
  virtual HomCoord& operator=(const HomCoord& h);

	friend std::ostream& operator<<(std::ostream& stream, const HomCoord& h);
		
 protected:
  HomCoord(float x = 0.0, float y = 0.0, float z = 0.0, float w = 0.0);
  HomCoord(const Vector& v);

	virtual std::ostream& addToStream(std::ostream& stream) const;
  Vector* m_vector;
};

} // end namespace fawkes

#endif /* __GEOMETRY_HOM_COORD_H_ */
