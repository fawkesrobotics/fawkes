
/***************************************************************************
 *  qa_omni_motion_model.cpp - OmniMotionModel QA
 *
 *  Generated: Thu April 03 14:29:33 2008
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

/// @cond QA

#include <plugins/navigator/robot_motion/linear_velocity_controller.h>
#include <utils/time/clock.h>

#include <unistd.h>
#include <cstdio>

int main(int argc, char** argv)
{
  Clock* clock = Clock::instance();
  
  LinearVelocityController lvc(clock, 20, 2.0);

  float vel = 0.0;
  float time = 0;
  float interval = 0.02;

  printf("Setting target vel to 2.0\n");
  lvc.set_target_velocity(2.0);
  for (unsigned int i = 0; i < 100; ++i)
    {
      lvc.set_actual_velocity(vel);
      vel = *( lvc.get_next_velocity() );
      time += interval;
      printf("%f %f\n", time, vel);
      usleep(20000);
    }
  lvc.set_actual_velocity(vel);
  printf("Setting target vel to 0.5\n");
  lvc.set_target_velocity(0.5);
  for (unsigned int i = 0; i < 65; ++i)
    {
      lvc.set_actual_velocity(vel);
      vel = *( lvc.get_next_velocity() );
      time += interval;
      printf("%f %f\n", time, vel);
      usleep(20000);
    }
  lvc.set_actual_velocity(vel);
  printf("Setting target vel to 2.5\n");
  lvc.set_target_velocity(2.5);
  for (unsigned int i = 0; i < 120; ++i)
    {
      lvc.set_actual_velocity(vel);
      vel = *( lvc.get_next_velocity() );
      time += interval;
      printf("%f %f\n", time, vel);
      usleep(20000);
    }
  lvc.set_actual_velocity(vel);
  printf("Setting target vel to 0.0\n");
  lvc.set_target_velocity(0.0);
  for (unsigned int i = 0; i < 100; ++i)
    {
      lvc.set_actual_velocity(vel);
      vel = *( lvc.get_next_velocity() );
      time += interval;
      printf("%f %f\n", time, vel);
      usleep(20000);
    }

  Clock::finalize();

  return 0;
}

/// @endcond
