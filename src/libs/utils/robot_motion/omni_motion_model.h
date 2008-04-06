
/***************************************************************************
 *  omni_motion_model.h - Motion model for a 3-wheeled omni-drive robot
 *
 *  Generated: Wed April 02 22:27:21 2008
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

#ifndef __UTILS_ROBOT_MOTION_OMNI_MOTION_MODEL_H_
#define __UTILS_ROBOT_MOTION_OMNI_MOTION_MODEL_H_

#include <utils/robot_motion/motion_model.h>
#include <geometry/matrix.h>

class OmniMotionModel : public MotionModel
{
 public:
  OmniMotionModel(float radius, float wheel_radius, float gear_reduction);
  virtual ~OmniMotionModel();

  void velocities_to_rpm(float vx, float vy, float omega,
			 float& rpm_right, float& rpm_rear, float& rpm_left);
  void pose_to_rpm(float dx, float dy, float dphi, float diff_sec,
		   float& rpm_right, float& rpm_rear, float& rpm_left);

  void update_odometry(float rot_right, float rot_rear, float rot_left,
		       float diff_sec);
  
  void get_odom_pose(float& x, float& y, float& phi);
  void get_odom_diff(float& dx, float& dy, float& dphi);
  void get_actual_velocities(float& vx, float& vy, float& omega);

  void reset_odometry();

 protected:
  void velocities_to_rpm(float* motion);
  void pose_to_rpm(float* pose, long diff_msec); 
  void update_odometry(float* odom, long diff_msec);

 private:
  Matrix m_omni_model;
  Matrix m_omni_model_inverse;

  float m_radius;
  float m_wheel_radius;
  float m_gear_reduction;

  float m_total_rotation;
};

#endif /* __UTILS_ROBOT_MOTION_OMNI_MOTION_MODEL_H_ */
