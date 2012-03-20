
/***************************************************************************
 *  roomba_plugin.cpp - Plugin to interface with a Roomba
 *
 *  Created: Thu Dec 30 22:05:07 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include "roomba_plugin.h"
#include "thread_roomba_500.h"
#include "sensor_thread.h"

using namespace fawkes;

/** @class RoombaPlugin "roomba_plugin.h"
 * Plugin to interface with a Roomba robot.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
RoombaPlugin::RoombaPlugin(Configuration *config)
  : Plugin(config)
{
  Roomba500Thread *roomba500_thread = new Roomba500Thread();
  thread_list.push_back(roomba500_thread);
  thread_list.push_back(new RoombaSensorThread(roomba500_thread));
}


PLUGIN_DESCRIPTION("Roomba vacuum robot plugin.")
EXPORT_PLUGIN(RoombaPlugin)
