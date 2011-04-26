
/***************************************************************************
 *  bb_handler.h - Joystick blackboard handler
 *
 *  Created: Tue Apr 26 18:26:29 2011
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

#ifndef __PLUGINS_JOYSTICK_BB_HANDLER_H_
#define __PLUGINS_JOYSTICK_BB_HANDLER_H_

class JoystickBlackBoardHandler
{
 public:
  virtual ~JoystickBlackBoardHandler();
  virtual void joystick_changed(unsigned int pressed_buttons, 
				float *axis_values) = 0;
  virtual void joystick_plugged(char num_axes, char num_buttons) = 0;
  virtual void joystick_unplugged() = 0;
};

#endif
