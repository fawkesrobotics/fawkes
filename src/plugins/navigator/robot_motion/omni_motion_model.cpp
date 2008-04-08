
/***************************************************************************
 *  omni_motion_model.cpp - Motion model for a 3-wheeled omni-drive robot
 *
 *  Generated: Wed April 02 22:59:35 2008
 *  Copyright  2008  Daniel Beck 
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <plugins/navigator/robot_motion/omni_motion_model.h>
#include <geometry/vector.h>
#include <geometry/hom_point.h>
#include <geometry/hom_vector.h>

#include <cmath>
#include <cstdio>
#include <cstring>

/** @class OmniMotionModel plugins/navigator/robot_motion/omni_motion_model.h
 * A motion model for a three-wheeled omni-drive robot. It is assumed that the wheels are
 * mounted at angles -60° (front right), 180° (rear), and 60° (left), all with the same
 * distance to the center of the robot.
 * @author Daniel Beck
 */

/** Constructor.
 * @param radius the distance of the wheels from the center of the robot
 * @param wheel_radius the radius of the wheels
 * @param gear_reduction the gear reduction factor. The shaft rotations of the motor have
 *                       to be multiplied with this factor to obtain the wheel rotaions. 
 * @param reverse_rotation set this to true if the gear inverses the direction of rotation
 *        of the wheel
 */
OmniMotionModel::OmniMotionModel(float radius, float wheel_radius, 
				 float gear_reduction, bool reverse_rotation)
  : MotionModel(3, 3),
    m_omni_model(3, 3),
    m_omni_model_inverse(3, 3)
{
  m_radius = radius;
  m_wheel_radius = wheel_radius;
  m_gear_reduction = gear_reduction;
  m_reverse_rotation = reverse_rotation;

  m_omni_model(0, 0) = 2.0 / sqrt(3.0); // == cos(30°)
  m_omni_model(0, 1) = 0.5;
  m_omni_model(0, 2) = m_radius / 1000;
  m_omni_model(1, 0) = 0.0;
  m_omni_model(1, 1) = -1.0;
  m_omni_model(1, 2) = m_radius / 1000;
  m_omni_model(2, 0) = -2.0 / sqrt(3.0); // == -cos(30°)
  m_omni_model(2, 1) = 0.5;
  m_omni_model(2, 2) = m_radius / 1000;
  
  m_omni_model /= ( m_wheel_radius / 1000 * 2.0 * M_PI );
  m_omni_model /= m_gear_reduction;

  if (m_reverse_rotation)
    { m_omni_model *= -1.0; }

  m_omni_model_inverse = m_omni_model.get_inverse();

  m_total_rotation = 0.0;
}

/** Destructor. */
OmniMotionModel::~OmniMotionModel()
{
}

/** Get RPMs for the three motor to move the robot with the specified velocities.
 * @param vx desired forward velocity in m/s
 * @param vy desired sidwared velocity in m/s
 * @param omega desired rotational velocity in rad/s
 * @param rpm_right rpm command for front right motor
 * @param rpm_rear rpm command for rear motor
 * @param rpm_left rpm command for front left motor
 */
void
OmniMotionModel::velocities_to_rpm(float vx, float vy, float omega,
				   float& rpm_right, float& rpm_rear, float& rpm_left)
{
  Vector velocity(3);
  velocity[0] = vx;
  velocity[1] = vy;
  velocity[2] = omega;

  velocities_to_rpm( velocity.data_ptr() );

  rpm_right = m_motor_rpm_cmds[0];
  rpm_rear  = m_motor_rpm_cmds[1];
  rpm_left  = m_motor_rpm_cmds[2];
}

/** Get RPMs for the three motors to move to the given positin within the given time.
 * @param dx desired x-offset in meters
 * @param dy desired y-offset in meters
 * @param dphi desired phi-offset in rad
 * @param diff_sec the time in seconds
 * @param rpm_right rpm command for front right motor
 * @param rpm_rear rpm command for rear motor
 * @param rpm_left rpm command for front left motor
 */
void
OmniMotionModel::pose_to_rpm(float dx, float dy, float dphi, float diff_sec,
			     float& rpm_right, float& rpm_rear, float& rpm_left)
{
  if ( (dx == 0.0 && dy == 0.0 && dphi == 0.0) || (diff_sec == 0.0) )
    { 
      bzero( m_motor_rpm_cmds, m_control_dimensions * sizeof(float) );
    }
  else
    {
      Vector pose(3);
      pose[0] = dx;
      pose[1] = dy;
      pose[2] = dphi;
      
      pose_to_rpm( pose.data_ptr(), (long) rint(diff_sec * 1000) );
    }
      
  rpm_right = m_motor_rpm_cmds[0];
  rpm_rear  = m_motor_rpm_cmds[1];
  rpm_left  = m_motor_rpm_cmds[2];
}

/** Update the odometry.
 * @param rot_right the number of shaft rotations of the front right motor
 * @param rot_rear the number of shaft rotations of the rear motor
 * @param rot_left the number of shaft rotations of the front left motor
 * @param diff_sec the time that passed since the last update in sec
 */
void
OmniMotionModel::update_odometry(float rot_right, float rot_rear, float rot_left, 
				 float diff_sec)
{
  Vector odom(3);
  odom[0] = rot_right;
  odom[1] = rot_rear;
  odom[2] = rot_left;
  long diff_msec = (long) rint(diff_sec * 1000);

  update_odometry(odom.data_ptr(), diff_msec);
}
  
/** Get the odometric pose estimation.
 * The pose is given wrt. the coordinate system that is defined by robot's pose
 * at the time of the last odometry reset.
 * @param x reference to the return variable for the x-coordinate of the robot's pose
 * @param y reference to the return variable for the y-coordinate of the robot's pose
 * @param phi reference to the return variable for the orientation of the robot's pose
 */
void
OmniMotionModel::get_odom_pose(float& x, float& y, float& phi)
{
  x   = m_odometric_pose[0];
  y   = m_odometric_pose[1];
  phi = m_odometric_pose[2];
}

/** Get the change of position since the last update.
 * @param dx reference to the return variable for the x-coordinate of the robot's last
 *        movement vector
 * @param dy reference to the return variable for the y-coordinate of the robot's last
 *        movement vector
 * @param dphi reference to the return variable for the orienation of the robot's last
 *        movement vector
 */
void
OmniMotionModel::get_odom_diff(float& dx, float& dy, float& dphi)
{
  dx   = m_last_movement[0];
  dy   = m_last_movement[1];
  dphi = m_last_movement[2];
}

/** Get the actual velcity vector.
 * @param vx the x-component of the actual velocity vector
 * @param vy the y-component of the actual velocity vector
 * @param omega the rotational component of the actual velocity vector
 */
void
OmniMotionModel::get_actual_velocities(float& vx, float& vy, float& omega)
{
  vx    = m_actual_velocities[0];
  vy    = m_actual_velocities[1];
  omega = m_actual_velocities[2];
}

void
OmniMotionModel::reset_odometry()
{
  m_total_rotation = 0.0;
  MotionModel::reset_odometry();
}

void
OmniMotionModel::velocities_to_rpm(float* velocities)
{
  Vector velocities_vector(m_motion_dimensions, velocities);
  Vector cmd_vector(m_control_dimensions, m_motor_rpm_cmds, false);
  cmd_vector = m_omni_model * velocities_vector;
  cmd_vector *= 60;
}

void
OmniMotionModel::pose_to_rpm(float* pose, long diff_msec)
{
  Vector p(m_motion_dimensions, pose);

  float s = sqrt( p[0] * p[0] + p[1] * p[1] );
  float r = 0.5 * s * 1.0 / sin( p[2] / 2.0 ); 
  float l = r * p[2];

  //printf("l=%.2f  phi=%.2f  r=%.2f  s=%.2f\n", l, p[2], r, s);
  
  HomVector v( p[0], p[1] );
  v.set_length(l);
  float angle = acos( sin( p[2] ) * r / s);
  angle *= (p[2] < 0) ? -1.0 : 1.0; 
  v.rotate_z( -angle );

  Vector vel(m_control_dimensions);
  vel[0] = v.x() * 1000.0 / diff_msec;
  vel[1] = v.y() * 1000.0 / diff_msec;
  vel[2] = p[2]  * 1000.0 / diff_msec;

  Vector cmd_vector(m_control_dimensions, m_motor_rpm_cmds, false);
  cmd_vector = m_omni_model * vel;
  cmd_vector *= 60;
}

void
OmniMotionModel::update_odometry(float* odom_diff, long diff_msec)
{
  Vector odom(m_control_dimensions, odom_diff);

  Vector velocity(m_motion_dimensions, m_actual_velocities, false);
  velocity = m_omni_model_inverse * odom;
  velocity /= diff_msec / 1000.0;
  //velocity.print_info("v");

  Vector motion(m_motion_dimensions, m_last_movement, false);
  motion[0] = velocity[0] * diff_msec / 1000.0;
  motion[1] = velocity[1] * diff_msec / 1000.0;
  motion[2] = velocity[2] * diff_msec / 1000.0;
  //motion.print_info("m");
  
  float l   = sqrt( motion[0] * motion[0] + motion[1] * motion[1] );
  float phi = motion[2];
  float r   = fabs( (phi > 1e-6) ? l / phi : 0.0 );
  float s   = 2.0 * r * sin( 0.5 * fabs(phi) );
  //printf("l=%.6f  phi=%.6f  r=%.6f  s=%.6f\n", l, phi, r, s);
  
  HomVector trans(motion[0], motion[1]);
  if (s != 0.0)
    {
      trans.set_length(s);
      float t = sin(phi) * r / s;
      float angle;
      if ( t < 0.0 )
	{ angle = M_PI; }
      else if ( t > 1.0 )
	{ angle = 0.0;}
      else
	{ angle = acos(t); }
      angle *= (phi < 0) ? -1.0 : 1.0; 
      trans.rotate_z( angle );
      motion[0] = trans.x();
      motion[1] = trans.y();
      //printf("angle=%.2f  trans=(%.6f, %.6f)\n", angle, trans.x(), trans.y());
    }
  //   else
  //     { printf("trans=(%.6f, %.6f)\n", trans.x(), trans.y()); }

  Vector pose(m_motion_dimensions, m_odometric_pose, false);
  trans.rotate_z(m_total_rotation);
  pose[0] += trans.x();
  pose[1] += trans.y();
  pose[2] += phi;

  m_total_rotation += phi;
}
