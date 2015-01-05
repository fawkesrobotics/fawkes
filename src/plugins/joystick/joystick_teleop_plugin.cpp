
/***************************************************************************
 *  joystick_teleop_plugin.cpp - Plugin for joystick remote control
 *
 *  Created: Sun Nov 13 23:20:35 2011 (as part of the robotino plugin)
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
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

#include <core/plugin.h>

#include "joystick_teleop_thread.h"

using namespace fawkes;

/** Plugin to remote control a robot using a joystick.
 * @author Tim Niemueller
 */
class JoystickTeleOpPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  JoystickTeleOpPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new JoystickTeleOpThread());
  }
};

PLUGIN_DESCRIPTION("Joystick remote control")
EXPORT_PLUGIN(JoystickTeleOpPlugin)
