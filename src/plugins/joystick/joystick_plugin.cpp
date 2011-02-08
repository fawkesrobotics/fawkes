
/***************************************************************************
 *  joystick_plugin.h - Fawkes Joystick Plugin
 *
 *  Created: Sat Nov 22 18:04:49 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <plugins/joystick/joystick_plugin.h>

#include "act_thread.h"
#include "sensor_thread.h"
#include "acquisition_thread.h"

using namespace fawkes;

/** @class JoystickPlugin "joystick_plugin.h"
 * Joystick plugin for Fawkes.
 * This plugin provides access to a joystick from within Fawkes.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
JoystickPlugin::JoystickPlugin(Configuration *config)
  : Plugin(config)
{
  JoystickAcquisitionThread *aqt = new JoystickAcquisitionThread();
  JoystickSensorThread *senst = new JoystickSensorThread(aqt);
  thread_list.push_back(senst);
  thread_list.push_back(aqt);
  thread_list.push_back(new JoystickActThread(aqt, senst));
}


PLUGIN_DESCRIPTION("Provides access to a joystick")
EXPORT_PLUGIN(JoystickPlugin)
