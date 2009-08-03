
/***************************************************************************
 *  line.h - A line
 *
 *  Created: Fri Sep 28 15:51:58 2007
 *  Copyright  2007  Daniel Beck
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

#ifndef __LINE_H_
#define __LINE_H_

#include <geometry/geom_obj.h>

namespace fawkes {

class Line : public GeomObj
{
 public:
  Line(const HomPoint& p, const HomVector& v);
  Line(const HomPoint& p1, const HomPoint& p2);
  
  virtual ~Line();

  Line& apply_transform(const HomTransform& t);
  Line& apply_transform_ref(const HomTransform& t);

  Line& trans(float trans_x, float trans_y, float trans_z);
  Line& trans_ref(float trans_x, float trans_y, float trans_z);

  Line& rotate_x(float angle);
  Line& rotate_x_ref(float angle);

  Line& rotate_y(float angle);
  Line& rotate_y_ref(float angle);

  Line& rotate_z(float angle);
  Line& rotate_z_ref(float angle);

  //  Line& rotate_x_ref(float angle, const HomPoint& p);
  
 private:
  HomPoint mBasePoint;
  HomVector mDirection;
};

} // end namespace fawkes

#endif /* __LINE_H_ */
