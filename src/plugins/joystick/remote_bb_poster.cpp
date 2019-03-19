
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
#include <interfaces/JoystickInterface.h>
#include <logging/logger.h>

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
JoystickRemoteBlackBoardPoster::JoystickRemoteBlackBoardPoster(const char *       host,
                                                               unsigned short int port,
                                                               Logger *           logger)
: logger_(logger)
{
	bb_ = new RemoteBlackBoard(host, port);

	joystick_if_     = bb_->open_for_writing<JoystickInterface>("Joystick");
	warning_printed_ = false;
}

/** Destructor. */
JoystickRemoteBlackBoardPoster::~JoystickRemoteBlackBoardPoster()
{
	bb_->close(joystick_if_);
	delete bb_;
}

void
JoystickRemoteBlackBoardPoster::joystick_changed(unsigned int pressed_buttons, float *axis_values)
{
	if (!bb_->is_alive()) {
		if (bb_->try_aliveness_restore()) {
			logger_->log_info("Joystick", "Connection re-established, writing data");
			warning_printed_ = false;
		}
	}

	try {
		joystick_if_->set_pressed_buttons(pressed_buttons);
		joystick_if_->set_axis(axis_values);
		joystick_if_->write();
	} catch (Exception &e) {
		if (!warning_printed_) {
			e.print_trace();
			logger_->log_warn("Joystick",
			                  "Lost connection to BlackBoard, "
			                  "will try to re-establish");
			warning_printed_ = true;
		}
	}
}

void
JoystickRemoteBlackBoardPoster::joystick_plugged(char num_axes, char num_buttons)
{
	joystick_if_->set_num_axes(num_axes);
	joystick_if_->set_num_buttons(num_buttons);
	joystick_if_->write();
}

void
JoystickRemoteBlackBoardPoster::joystick_unplugged()
{
	joystick_if_->set_num_axes(0);
	joystick_if_->set_num_buttons(0);
	joystick_if_->write();
}
