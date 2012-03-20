
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

#include "remote_bb_poster.h"

#include <blackboard/remote.h>
#include <logging/logger.h>
#include <interfaces/JoystickInterface.h>

using namespace fawkes;

/** @class JoystickRemoteBlackBoardPoster "remote_bb_poster.h"
 * Glue to post new data to a RemoteBlackBoard.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param host remote bb host to connect to
 * @param port remote bb port to connect to
 * @param logger logger
 */
JoystickRemoteBlackBoardPoster::JoystickRemoteBlackBoardPoster(const char *host,
							 unsigned short int port,
							 Logger *logger)
  : __logger(logger)
{
  __bb = new RemoteBlackBoard(host, port);

  __joystick_if = __bb->open_for_writing<JoystickInterface>("Joystick");
  __warning_printed = false;
}

/** Destructor. */
JoystickRemoteBlackBoardPoster::~JoystickRemoteBlackBoardPoster()
{
  __bb->close(__joystick_if);
  delete __bb;
}

void
JoystickRemoteBlackBoardPoster::joystick_changed(unsigned int pressed_buttons,
						 float *axis_values)
{
  if ( ! __bb->is_alive() ) {
    if ( __bb->try_aliveness_restore() ) {
      __logger->log_info("Joystick", "Connection re-established, writing data");
      __warning_printed = false;
    }
  }
  
  try {
    __joystick_if->set_pressed_buttons(pressed_buttons);
    __joystick_if->set_axis(axis_values);
    __joystick_if->write();
  } catch (Exception &e) {
    if ( ! __warning_printed ) {
      e.print_trace();
      __logger->log_warn("Joystick", "Lost connection to BlackBoard, "
			 "will try to re-establish");
      __warning_printed = true;
    }
  }
}

void
JoystickRemoteBlackBoardPoster::joystick_plugged(char num_axes, char num_buttons)
{
  __joystick_if->set_num_axes( num_axes );
  __joystick_if->set_num_buttons( num_buttons );
  __joystick_if->write();
}

void
JoystickRemoteBlackBoardPoster::joystick_unplugged()
{
  __joystick_if->set_num_axes( 0 );
  __joystick_if->set_num_buttons( 0 );
  __joystick_if->write();
}
