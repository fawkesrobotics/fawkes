
/***************************************************************************
 *  force_feedback.h - Force feedback for joysticks using Linux input API
 *
 *  Created: Sun Feb 06 23:50:57 2011 (Super Bowl XLV)
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_JOYSTICK_FORCE_FEEDBACK_H_
#define __PLUGINS_JOYSTICK_FORCE_FEEDBACK_H_

#include <stdint.h>
#include <linux/input.h>

class JoystickForceFeedback
{
 public:
  /** Direction of the effect. */
  typedef enum {
    DIRECTION_DOWN  = 0x0000,	/**< Downward effect direction. */
    DIRECTION_LEFT  = 0x4000,	/**< Left effect direction. */
    DIRECTION_UP    = 0x8000,	/**< Upward effect direction. */
    DIRECTION_RIGHT = 0xC000	/**< Right effect direction. */
  } Direction;

  JoystickForceFeedback(const char *device_name);
  ~JoystickForceFeedback();

  void rumble(uint16_t strong_magnitude, uint16_t weak_magnitude,
	      Direction direction = DIRECTION_DOWN,
	      uint16_t length = 0, uint16_t delay = 0);

  void stop_all();
  void stop_rumble();

  bool is_rumbling() { return (__rumble.id != -1); }
  bool can_rumble()   { return __can_rumble;   }
  bool can_periodic() { return __can_periodic; }
  bool can_constant() { return __can_constant; }
  bool can_spring()   { return __can_spring;   }
  bool can_friction() { return __can_friction; }
  bool can_damper()   { return __can_damper;   }
  bool can_inertia()  { return __can_inertia;  }
  bool can_ramp()     { return __can_ramp;     }
  bool can_square()   { return __can_square;   }
  bool can_triangle() { return __can_triangle; }
  bool can_sine()     { return __can_sine;     }
  bool can_saw_up()   { return __can_saw_up;   }
  bool can_saw_down() { return __can_saw_down; }
  bool can_custom()   { return __can_custom;   }

 private:
  int __fd;
  struct ff_effect __rumble;

  int  __num_effects;

  bool __can_rumble;
  bool __can_periodic;
  bool __can_constant;
  bool __can_spring;
  bool __can_friction;
  bool __can_damper;
  bool __can_inertia;
  bool __can_ramp;
  bool __can_square;
  bool __can_triangle;
  bool __can_sine;
  bool __can_saw_up;
  bool __can_saw_down;
  bool __can_custom;

};


#endif
