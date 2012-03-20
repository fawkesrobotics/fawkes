
/***************************************************************************
 *  bb_handler.cpp - Joystick blackboard handler
 *
 *  Created: Tue Apr 26 18:27:29 2011
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

#include "bb_handler.h"

/** @class JoystickBlackBoardHandler "acquisition_thread.h"
 * Handler class for joystick data.
 * This interface allows to plug a generic handler to the
 * JoystickAcquisitionThread via the alternative constructor. This can be
 * used to directly instantiate the acquisition thread outside of Fawkes.
 * @author Tim Niemueller
 *
 * @fn void JoystickBlackBoardHandler::joystick_changed(unsigned int pressed_buttons, float *axis_values) = 0
 * Joystick data changed.
 * @param pressed_buttons the new pressed_buttons array
 * @param axis_values array of axis values, the length is at least num_axes()
 *
 * @fn void JoystickBlackBoardHandler::joystick_plugged(char num_axes, char num_buttons)
 * A (new) joystick has been plugged in
 * @param num_axes number of axes
 * @param num_buttons number of buttons
 *
 * @fn void JoystickBlackBoardHandler::joystick_unplugged()
 * The joystick has been unplugged and is no longer available.
 */

/** Virtual empty destructor. */
JoystickBlackBoardHandler::~JoystickBlackBoardHandler()
{
}
