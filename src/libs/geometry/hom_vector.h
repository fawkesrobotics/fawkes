
/***************************************************************************
 *  hom_vector.h - Homogenous vector
 *
 *  Created: Wed Sep 26 16:58:51 2007
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

#ifndef __GEOMETRY_HOM_VECTOR_H_
#define __GEOMETRY_HOM_VECTOR_H_

#include <geometry/hom_coord.h>

class HomVector : public HomCoord
{
 public:
  HomVector(float x = 0, float y = 0, float z = 0);
  HomVector(const HomCoord& h);
  ~HomVector();

  float length() const;
  HomVector& unit();
  HomVector& set_length(float length);
  HomVector& scale(float factor);

/*  private: */
/*   float m_length; */
/*   bool  m_changed; */
};

#endif /* __GEOMETRY_HOM_VECTOR_H_ */
