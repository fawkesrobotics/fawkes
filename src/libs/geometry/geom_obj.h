
/***************************************************************************
 *  geom_obj.h - Geometric Object
 *
 *  Created: Fri Sep 28 10:03:45 2007
 *  Copyright  2007  Daniel Beck
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

#ifndef __GEOMETRY_GEOM_OBJ_H_
#define __GEOMETRY_GEOM_OBJ_H_

#include <geometry/hom_point.h>
#include <geometry/hom_vector.h>
#include <geometry/hom_transform.h>

#include <vector>

class GeomObj
{
 public:
  virtual ~GeomObj();

 protected:
  GeomObj(const HomTransform& t, const HomPoint& ref_point);

  GeomObj& _apply_transform(const HomTransform& t);
  GeomObj& _apply_transform_ref(const HomTransform& t);

  GeomObj& _rotate_x(float angle);
  GeomObj& _rotate_y(float angle);
  GeomObj& _rotate_z(float angle);

  GeomObj& _rotate_x_ref(float angle);
  GeomObj& _rotate_y_ref(float angle);
  GeomObj& _rotate_z_ref(float angle);

  GeomObj& _trans(float x, float y, float z);
  GeomObj& _trans_ref(float x, float y, float z);

  HomPoint _get_refpoint_ref() const;
  
  std::vector<HomVector> _get_vectors_local() const;
  std::vector<HomVector> _get_vectors_ref();

  HomTransform mToRefCS;
  HomPoint mRefPoint;
  std::vector<HomVector> mVectorsLocal;
  std::vector<HomVector> mVectorsRef;

  bool mChanged;
};

#endif /* __GEOMETRY_GEOM_OBJ_H_ */
