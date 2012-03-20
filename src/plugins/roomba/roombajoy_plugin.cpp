
/***************************************************************************
 *  roombajoy_plugin.cpp - Control your Roomba with a joystick
 *
 *  Created: Sat Jan 29 14:30:07 2011
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

#include "roombajoy_plugin.h"
#include "roombajoy_thread.h"

using namespace fawkes;

/** @class RoombaJoystickPlugin "roombajoy_plugin.h"
 * Control Roomba with a Joystick.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
RoombaJoystickPlugin::RoombaJoystickPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new RoombaJoystickThread());
}


PLUGIN_DESCRIPTION("Control your Roomba with a Joystick")
EXPORT_PLUGIN(RoombaJoystickPlugin)
