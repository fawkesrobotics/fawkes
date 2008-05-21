
/***************************************************************************
 *  joystick_control.h - Joystick Control
 *
 *  Generated: Tue Jun 05 14:52:10 2007
 *  Copyright  2007  Martin Liebenberg
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
 
#ifndef __NAVIGATOR_JOYSTICK_CONTROL_H_
#define __NAVIGATOR_JOYSTICK_CONTROL_H_

#include <utils/time/time.h>

namespace fawkes {
  class MotorInterface;
  class Logger;
  class Configuration;
  class KickerInterface;
  class Clock;
}

class JoystickControl
{
 public:
  JoystickControl(fawkes::MotorInterface *motor_interface,
		  fawkes::KickerInterface *kicker_interface, 
		  fawkes::Logger *logger,
		  fawkes::Configuration *config,
		  fawkes::Clock *clock);
  virtual ~JoystickControl();
    
  void enqueueCommand(double forward, double sideward, double rotation, double max_velocity);

  void enqueueKick(bool left, bool center, bool right);
 private:
  fawkes::Logger *logger;
  fawkes::Configuration *config;

  double actual_velocity;
  double last_joystick_axis_scale;
  double actual_rotation_scale;
    
  fawkes::MotorInterface *motor_interface;
  fawkes::KickerInterface *kicker_interface;
  fawkes::Clock *clock;
  fawkes::Time last_kick_time;
        
  unsigned int logger_modulo_counter;
  unsigned int logger_modulo_counter2;

  float joystick_max_acceleration;
  float joystick_max_rotation;
  float joystick_max_velocity;
};


#endif /*JOYSTICK_CONTROL_H_*/
