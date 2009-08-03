
/***************************************************************************
 *  hom_pose_2d.h - 2-dimensional Homogenous Pose
 *
 *  Created: Fri Oct 10 11:06:45 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __GEOMETRY_HOM_POSE_2D_H_
#define __GEOMETRY_HOM_POSE_2D_H_

#include <geometry/transformable.h>

namespace fawkes {
class HomPoint;
class HomVector;

class HomPose2d : public Transformable
{
 public:
  HomPose2d(const HomPoint& pos, const HomVector& orientation);
  HomPose2d(float x = 0.0, float y = 0.0, float yaw = 0.0);
  HomPose2d(const HomPose2d& p);
  ~HomPose2d();

  const HomPose2d& operator=(const HomPose2d& p);

  float x() const;
  void  x(float x);

  float y() const;
  void  y(float y);

  float yaw() const;
  void  yaw(float yaw);

  const HomPoint&  position() const;
  const HomVector& orientation() const;

  void set_position(const HomPoint& p);

 protected:
  void register_primitives();
  void post_transform();

 private:
  HomPoint*  m_position;
  HomVector* m_orientation;

  float m_yaw;
};

} // end namespace fawkes

#endif /*  __GEOMETRY_HOM_POSE_2D_H_ */
