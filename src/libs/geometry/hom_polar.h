
/***************************************************************************
 *  hom_polar.h - A polar coordinate
 *
 *  Created: Tue April 22 22:42:38 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __GEOMETRY_HOM_POLAR_H_
#define __GEOMETRY_HOM_POLAR_H_

#include <geometry/hom_vector.h>

namespace fawkes {

class HomPolar : public HomVector
{
 public:
  HomPolar(float r = 0.0, float phi = 0.0);
  HomPolar(float r, float phi_x, float phi_y);
  HomPolar(const HomCoord& h);
  virtual ~HomPolar();

  float  r() const;
  void   r(float r);

  float  phi() const;
  void   phi(float phi);
  float  phi_z() const;
  void   phi_z(float phi_z);
  float  phi_y() const;
  void   phi_y(float phi_y);
  void   phi(float phi_x, float phi_y);

  virtual HomPolar& rotate_x(float rad);
  virtual HomPolar& rotate_y(float rad);
  virtual HomPolar& rotate_z(float rad);
  
  virtual HomPolar  operator-(const HomPolar& h) const;
  virtual HomPolar& operator-=(const HomPolar& h);

  virtual HomPolar  operator+(const HomPolar& h) const;
  virtual HomPolar& operator+=(const HomPolar& h);

  virtual HomPolar& operator=(const HomPolar& h);
  
  HomVector get_vector() const;

 private:
  float m_r;
  float m_phi_z;
  float m_phi_y;
};

} // end namespace fawkes

#endif /* __GEOMETRY_HOM_POLART_H_ */
