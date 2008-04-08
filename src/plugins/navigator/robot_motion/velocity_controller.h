
/***************************************************************************
 *  velocity_controller.h - Velocity controller interface
 *
 *  Generated: Wed April 03 00:04:26 2008
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

#ifndef __PLUGINS_NAVIGATOR_ROBOT_MOTION_VELOCITY_CONTROLLER_H_
#define __PLUGINS_NAVIGATOR_ROBOT_MOTION_VELOCITY_CONTROLLER_H_

#include <utils/time/clock.h>
#include <utils/time/time.h>

class VelocityController
{
 public:
  VelocityController(Clock* clock, unsigned int avg_loop_time_ms,
		     unsigned int dimensions);
  virtual ~VelocityController();

  void set_target_velocity(float* target);
  void set_actual_velocity(float* actual);
  
  virtual float* get_next_velocity()                                         = 0;
  virtual float* get_next_velocity(float dist_to_target)                     = 0;

 protected:
  Clock* m_clock;
  long m_avg_loop_time;
  unsigned int m_dimensions;

  float* m_target_velocity;
  float* m_actual_velocity;
  float* m_next_velocity;
  float* m_start_vel_diff;

  Time m_last_loop;
  
 private:
  bool m_first;
  bool m_set_vel_diff;
};

#endif /* __PLUGINS_NAVIGATOR_ROBOT_MOTION_VELOCITY_CONTROLLER_H_ */

