
/***************************************************************************
 *  motion_model.cpp - Motion model interface
 *
 *  Generated: Wed April 02 23:43:34 2008
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

#include <plugins/navigator/robot_motion/motion_model.h>

#include <cstdlib>
#include <cstring>

/** @class MotionModel plugins/navigator/robot_motion/motion_model.h
 * A base class for motion models. Most of the methods are protected since it is intended
 * that better interface methods are provided by derived classes.
 * @author Daniel Beck
 */

/** @fn void MotionModel::velocities_to_rpm(float* velocities) = 0
 * This method has to compute the RPMs for the given velocities and store them in the
 * array m_motor_rpm_cmds.
 * @param velocities array containing the desired velocities
 */

/** @fn void MotionModel::pose_to_rpm(float* pose, long diff_msec) = 0
 * This method has to compute the RPMs in order to reach to the given pose in the given
 * time and store them in the array m_motor_rpm_cmds.
 * @param pose array containg the desired pose offset
 * @param diff_msec the the time in which the target location has to be reached in 
 *        milliseconds
 */

/** @fn void MotionModel::update_odometry(float* odom_diff, long diff_msec) = 0
 * The new actual velocities and the odometric pose have to be computed here based on the
 * on the rotations of the wheels since the last update. The results have to be written
 * to m_actual_velocities, m_odometric_pose, and m_lost_movement.
 * @param odom_diff changes of odometry since last update
 * @param diff_msec time since last update in milliseconds
 */

/** @var MotionModel::m_actual_velocities
 * This array holds the current velocities.
 */

/** @var MotionModel::m_odometric_pose
 * This array holds the odometric pose wrt. to pose at last reset.
 */

/** @var MotionModel::m_last_movement
 * The movement in the last step.
 */

/** @var MotionModel::m_motor_rpm_cmds
 * This array holds the caclulated RPMs which are sent to the motors.
 */

/** @var MotionModel::m_motion_dimensions
 * The number of dimensions that are necessary to specify the the (spatial) configuration
 * of the robot.
 */

/** @var MotionModel::m_control_dimensions
 * The dimension of the control parameter space.
 */

/** Constructor.
 * @param motion_dimensions the number of dimensions that are necessary to specify the
 *        the (spatial) configuration of the robot
 * @param control_dimensions the dimension of the control parameter space
 */
MotionModel::MotionModel(unsigned int motion_dimensions,
			 unsigned int control_dimensions)
{
  m_motion_dimensions = motion_dimensions;
  m_control_dimensions = control_dimensions;

  m_actual_velocities  = (float*) malloc( m_motion_dimensions * sizeof(float) );
  m_last_movement      = (float*) malloc( m_motion_dimensions * sizeof(float) );
  m_odometric_pose     = (float*) malloc( m_motion_dimensions * sizeof(float) );
  m_motor_rpm_cmds     = (float*) malloc( m_control_dimensions * sizeof(float) );

  memset( m_actual_velocities, 0, m_motion_dimensions * sizeof(float) );
  memset( m_last_movement, 0, m_motion_dimensions * sizeof(float) );
  memset( m_odometric_pose, 0, m_motion_dimensions * sizeof(float) );
  memset( m_motor_rpm_cmds, 0, m_control_dimensions * sizeof(float) );
}

/** Destructor. */
MotionModel::~MotionModel()
{
  free(m_actual_velocities);
  free(m_last_movement);
  free(m_odometric_pose);
  free(m_motor_rpm_cmds);
}

/** Reset the odometry. */
void
MotionModel::reset_odometry()
{
  memset( m_odometric_pose, 0, m_motion_dimensions * sizeof(float) );
}

/** Get the motor RPMs for to realize the priorily specified motion (either by
 * velocities_to_rpm or by pose_to_rpm).
 * @return an array containing the motor commands
 */
float*
MotionModel::get_rpm_cmds()
{
  return m_motor_rpm_cmds;
}

/** Get the current odometric pose estimation wrt. the coordinate system defined by the
 * robot's pose at the last reset of the odometry.
 * @return an array containing the current pose of the robot
 */
float*
MotionModel::get_odom_pose()
{
  return m_odometric_pose;
}

/** Get the robot's movement during the last step.
 * @return an array describing the last move
 */
float*
MotionModel::get_odom_diff()
{
  return m_last_movement;
}

/** Get the actual velocities.
 * @return an array containing the actual velocities
 */
float*
MotionModel::get_actual_velocities()
{
  return m_actual_velocities;
}
