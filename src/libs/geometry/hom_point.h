
/***************************************************************************
 *  hom_point.h - Homogenous point
 *
 *  Created: Thu Sep 27 16:55:50 2007
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

#ifndef __GEOMETRY_HOM_POINT_H_
#define __GEOMETRY_HOM_POINT_H_

#include <geometry/hom_coord.h>

class HomVector;

class HomPoint : public HomCoord
{
 public:
  HomPoint(float x = 0.0, float y = 0.0, float z = 0.0);
  HomPoint(const HomCoord& h);
  ~HomPoint();

  float distance() const;

  HomPoint& move(float dx, float dy, float dz);
  HomPoint& move_to(float x, float y, float z);
};

#endif /* __GEOMETRY_HOM_POINT_H_ */
