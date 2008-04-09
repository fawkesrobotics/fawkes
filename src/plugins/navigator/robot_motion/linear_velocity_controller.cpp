
/***************************************************************************
 *  linear_velocity_controller.cpp - Linear velocity controller
 *
 *  Generated: Wed April 03 00:31:38 20 
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

#include <plugins/navigator/robot_motion/linear_velocity_controller.h>
#include <plugins/navigator/robot_motion/pid_controller.h>
#include <cmath>
#include <cstdio>

/** @class LinearVelocityController plugins/navigator/robot_motion/linear_velocity_controller.h
 * A simple one-dimensional velocity controller with integrated PID controller that
 * accelerates/decelerates linearly.
 * @author Daniel Beck
 */

/** Constructor. 
 * @param clock pointer to a clock. 
 * @param avg_loop_time_ms the initial average loop time in milliseconds. 
 * @param max_accel  maximal accelation/deceleration in m/s
 */
LinearVelocityController::LinearVelocityController(Clock* clock, 
						   unsigned int avg_loop_time_ms,
						   float max_accel)
  : VelocityController(clock, avg_loop_time_ms, 1),
    m_start(clock)
{
  m_max_accel = max_accel;
  m_max_decel = max_accel;

  m_stopping = false;

  m_pid_controller = new PidController();
  m_pid_controller->set_coefficients(0.08, 0.08, 0.01);
}

/** Constructor.
 * @param clock pointer to a clock
 * @param avg_loop_time_ms the inititial average loop time in milliseconds 
 * @param max_accel maximal accelation in m/s
 * @param max_decel maximal decelerration in m/s
 */
LinearVelocityController::LinearVelocityController(Clock* clock,
						   unsigned int avg_loop_time_ms,
						   float max_accel,
						   float max_decel)
  : VelocityController(clock, avg_loop_time_ms, 1),
    m_start(clock)
{
  m_max_accel = max_accel;
  m_max_decel = max_decel;

  m_stopping = false;

  m_pid_controller = new PidController();
  m_pid_controller->set_coefficients(1.0, 0.0, 0.0);
}

/** Destructor. */
LinearVelocityController::~LinearVelocityController()
{
  delete m_pid_controller;
}

/** Convenience wrapper method. 
 * @param target target velocity.
 */
void
LinearVelocityController::set_target_velocity(float target)
{
  m_stopping = false;
  m_start.stamp();
  VelocityController::set_target_velocity(&target);
}

/** Convenience wrapper method.
 * @param actual the actual velocity. 
 */
void
LinearVelocityController::set_actual_velocity(float actual)
{
  VelocityController::set_actual_velocity(&actual);
}

float*
LinearVelocityController::get_next_velocity()
{
  if (*m_start_vel_diff == 0.0)
    { 
      *m_next_velocity = *m_target_velocity;
      return m_next_velocity;
    }

  Time next_loop(m_clock);
  next_loop.stamp();
  next_loop += m_avg_loop_time;

  float vel_diff;
  float attenuation;
  
  vel_diff = (*m_target_velocity - *m_actual_velocity) / *m_start_vel_diff;
  attenuation = sqrt( fabs(vel_diff) );
  
  float desired_velocity;
  if (vel_diff > 0.0)
    {
      desired_velocity = *m_target_velocity - *m_start_vel_diff 
	+ (next_loop - m_start).in_sec() * m_max_accel;

      if (desired_velocity > *m_target_velocity)
	{ desired_velocity = *m_target_velocity; }
    }
  else
    {
      desired_velocity = *m_target_velocity + *m_start_vel_diff 
	- (next_loop - m_start).in_sec() * m_max_accel;
      if (desired_velocity < *m_target_velocity)
	{ desired_velocity = *m_target_velocity; }
    }

  //  *m_next_velocity += (desired_velocity - *m_actual_velocity) * fabs(vel_diff);
  float error = desired_velocity - *m_actual_velocity;
  float timestep = m_avg_loop_time / 1000000.0;
  m_pid_controller->record(error, timestep);
  *m_next_velocity += m_pid_controller->get_output();
//   printf("desired=%f  actual=%f  error=%f  timestep=%f  output=%f\n", 
// 	 desired_velocity, *m_actual_velocity, error, timestep, *m_next_velocity);

  if (*m_next_velocity < 0)
    { *m_next_velocity = 0; }
  
//   printf("vel_target=%f  vel_act=%f  vel_diff=%f  vel_des=%f  vel_next=%f\n", 
//         *m_target_velocity, *m_actual_velocity, vel_diff, desired_velocity, 
//         *m_next_velocity);

  return m_next_velocity;
}

float*
LinearVelocityController::get_next_velocity(float dist_to_target)
{
  // compute how long it takes to slow down to a stop
  float stop_time_sec = *m_actual_velocity / m_max_decel;

  // compute distance travelled in this time
  float stop_dist = 0.5 * *m_actual_velocity * stop_time_sec;

  if ( !m_stopping && (stop_dist <= dist_to_target) )
    { 
      m_stopping = true;
      set_target_velocity(0.0); 
    }
  
  return get_next_velocity();
}
