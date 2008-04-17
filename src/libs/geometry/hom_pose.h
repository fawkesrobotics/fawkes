
/***************************************************************************
 *  hom_pose.h - Homogenous Pose
 *
 *  Created: Sun April 13 16:10:45 2008
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

#ifndef __GEOMETRY_HOM_POSE_H_
#define __GEOMETRY_HOM_POSE_H_

#include <geometry/hom_point.h>

class HomPose : public HomPoint
{
 public:
  HomPose(float x = 0, float y = 0, float yaw = 0);
  HomPose( float x, float y, float z, 
	   float roll, float pitch, float yaw );
  HomPose(const HomCoord& h);
  virtual ~HomPose();
  
  float  roll() const;
  float& roll();
  void   roll(float roll);

  float  pitch() const;
  float& pitch();
  void   pitch(float pitch);

  float  yaw() const;
  float& yaw();
  void   yaw(float yaw);

  HomPoint pos() const;

  virtual HomPose& rotate_x(float rad);
  virtual HomPose& rotate_y(float rad);
  virtual HomPose& rotate_z(float rad);

 private:
  float m_roll;
  float m_pitch;
  float m_yaw;
};

#endif /* __GEOMETRY_HOM_POSE_H_ */
