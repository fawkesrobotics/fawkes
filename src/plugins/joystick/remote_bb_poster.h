
/***************************************************************************
 *  remote_bb_poster.h - Joystick handler writing to remote blackboard
 *
 *  Created: Sat Jan 29 12:10:53 2011
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

#ifndef __PLUGINS_JOYSTICK_REMOTE_BB_POSTER_H_
#define __PLUGINS_JOYSTICK_REMOTE_BB_POSTER_H_

#include "acquisition_thread.h"
#include <utils/system/argparser.h>

namespace fawkes {
  class Logger;
  class BlackBoard;
  class JoystickInterface;
}

class JoystickRemoteBlackBoardPoster : public JoystickBlackBoardHandler
{
 public:
  JoystickRemoteBlackBoardPoster(const char *host, unsigned short int port,
				 fawkes::Logger *logger);
  ~JoystickRemoteBlackBoardPoster();

  /** Get blackboard.
   * @return blackboard instance
   */
  fawkes::BlackBoard * blackboard() { return __bb; }

  /** Get joystick interface.
   * @return joystick interface
   */
  fawkes::JoystickInterface * joystick_if() { return __joystick_if; }
  

  virtual void joystick_changed(unsigned int pressed_buttons, float *axis_values);
  virtual void joystick_plugged(char num_axes, char num_buttons);
  virtual void joystick_unplugged();

 private:
  bool __warning_printed;
  fawkes::BlackBoard *__bb;
  fawkes::JoystickInterface *__joystick_if;
  fawkes::Logger *__logger;
};

#endif
