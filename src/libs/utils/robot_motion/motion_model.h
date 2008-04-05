
/***************************************************************************
 *  motion_model.h - Motion model interface
 *
 *  Generated: Wed April 02 22:04:38 2008
 *  Copyright  2008  Daniel Beck 
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_ROBOT_MOTION_MOTION_MODEL_H_
#define __UTILS_ROBOT_MOTION_MOTION_MODEL_H_

class MotionModel
{
 public:
  MotionModel(unsigned int motion_dimensions, unsigned int control_dimensions);
  virtual ~MotionModel();

  virtual void reset_odometry();

 protected:
  virtual void velocities_to_rpm(float* velocities)                        = 0;
  virtual void pose_to_rpm(float* pose, long diff_msec)                    = 0;

  virtual void update_odometry(float* odom_diff, long diff_msec)           = 0;

  float* get_rpm_cmds();
  float* get_odom_pose();
  float* get_odom_diff();
  float* get_actual_velocities();

  float* m_actual_velocities;
  float* m_odometric_pose;
  float* m_last_movement;
  float* m_motor_rpm_cmds;

  unsigned int m_motion_dimensions;
  unsigned int m_control_dimensions;
};

#endif /* __UTILS_ROBOT_MOTION_MOTION_MODEL_H_ */

