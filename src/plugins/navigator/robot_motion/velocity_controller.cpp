
/*************************************************************************** 
 *  velocity_controller.cpp - Velocity controller
 *
 *  Generated: Wed April 03 00:26:40 2008
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <plugins/navigator/robot_motion/velocity_controller.h>
#include <cstdlib>
#include <cstring>
#include <cmath>

/** @class VelocityController plugins/navigator/robot_motion/velocity_controller.h
 * Velocity controller base class.
 * @author Daniel Beck
 */

/** @fn float* VelocityController::get_next_velocity()
 * @return array containing the next velocities to be sent to the motors
 */

/** @fn float* VelocityController::get_next_velocity(float dist_to_target)
 * @param dist_to_target distance to the next target point
 * @return array containing the next velocities to be sent to the motors
 */

/** @var VelocityController::m_clock
 * Pointer to a clock.
 */

/** @var VelocityController::m_avg_loop_time
 * Average loop time in micro seconds.
 */

/** @var VelocityController::m_dimensions
 * The number of dimensions of the control problem.
 */

/** @var VelocityController::m_target_velocity
 * The target velocity array.
 */

/** @var VelocityController::m_actual_velocity
 * The actual velocity array.
 */

/** @var VelocityController::m_next_velocity
 * The next velocity array.
 */

/** @var VelocityController::m_start_vel_diff
 * The difference of the actual velocity and the target velocity when the target velocity
 * was set the last time.
 */

/** @var VelocityController::m_last_loop
 * The time when the last loop ran.
 */

/** Constructor.
 * @param clock a clock
 * @param avg_loop_time_ms the average loop time in milliseconds
 * @param dimensions the number of dimensions of the control problem
 */
VelocityController::VelocityController(Clock* clock, 
				       unsigned int avg_loop_time_ms,
				       unsigned int dimensions)
  : m_last_loop(clock)
{
  m_clock = clock;
  m_avg_loop_time = avg_loop_time_ms * 1000;
  m_dimensions = dimensions;
  
  size_t array_size = m_dimensions * sizeof(float);
  m_target_velocity = (float*) malloc(array_size);
  m_next_velocity = (float*) malloc(array_size);
  m_actual_velocity = (float*) malloc(array_size);
  m_start_vel_diff = (float*) malloc(array_size);

  memset( m_start_vel_diff, 0, m_dimensions * sizeof(float) );
  memset( m_actual_velocity, 0, m_dimensions * sizeof(float) );
  memset( m_target_velocity, 0, m_dimensions * sizeof(float) );
  memset( m_next_velocity, 0, m_dimensions * sizeof(float) );

  m_first = true;
  m_set_vel_diff = true;
}

/** Desctructor. */
VelocityController::~VelocityController()
{
  free(m_target_velocity);
  free(m_actual_velocity);
  free(m_next_velocity);
  free(m_start_vel_diff);
}

/** Set the target velocities.
 * @param target array containing the target velocities
 */
void
VelocityController::set_target_velocity(float* target)
{
  memcpy( m_target_velocity, target, m_dimensions * sizeof(float) );
  m_set_vel_diff = true;
}

/** Set the actual velocities.
 * @param actual array containing the actual velocities
 */
void
VelocityController::set_actual_velocity(float* actual)
{
  if (m_first)
    { 
      m_last_loop.stamp();
      m_first = false;
    }
  else
    {
      Time now(m_clock);
      now.stamp();
      Time loop_time = now - m_last_loop;
      m_last_loop = now;
      m_avg_loop_time = (long) rint(0.9 * m_avg_loop_time + 0.1 * loop_time.in_usec());
    }
  
  memcpy( m_actual_velocity, actual, m_dimensions * sizeof(float) );
  if (m_set_vel_diff)
    {
      for (unsigned int i = 0; i < m_dimensions; ++i)
	{
	  m_start_vel_diff[i] = fabs(m_target_velocity[i] - m_actual_velocity[i]);

	  m_set_vel_diff = false;
	}
    }
}

