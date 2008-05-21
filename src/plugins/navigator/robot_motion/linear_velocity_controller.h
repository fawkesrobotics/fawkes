
/*************************************************************************** 
 *  linear_velocity_controller.h - Linear velocity controller
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

#ifndef __PLUGINS_NAVIGATOR_ROBOT_MOTION_LINEAR_VELOCITY_CONTROLLER_H_
#define __PLUGINS_NAVIGATOR_ROBOT_MOTION_LINEAR_VELOCITY_CONTROLLER_H_

#include <plugins/navigator/robot_motion/velocity_controller.h>
#include <utils/time/time.h>

class PidController;

class LinearVelocityController : public VelocityController
{
 public:
  LinearVelocityController(fawkes::Clock* clock,
			   unsigned int avg_loop_time_ms,
			   float max_accel);
  LinearVelocityController(fawkes::Clock* clock, unsigned int avg_loop_time_ms,
			   float max_accel, float max_decel);
  ~LinearVelocityController();

  void set_target_velocity(float target);
  void set_actual_velocity(float actual);

  float* get_next_velocity();
  float* get_next_velocity(float dist_to_target);

 private:
  float m_max_accel;
  float m_max_decel;

  fawkes::Time m_start;

  bool m_stopping;

  PidController* m_pid_controller;
};

#endif /* __PLUGINS_NAVIGATOR_ROBOT_MOTION_LINEAR_VELOCITY_CONTROLLER_H_ */
