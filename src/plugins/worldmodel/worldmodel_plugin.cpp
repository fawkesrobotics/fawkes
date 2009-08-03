
/***************************************************************************
 *  worldmodel_plugin.cpp - Fawkes WorldModel Plugin
 *
 *  Created: Fri Jun 29 11:47:53 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <plugins/worldmodel/worldmodel_plugin.h>
#include <plugins/worldmodel/wm_thread.h>
#include <plugins/worldmodel/net_thread.h>

using namespace fawkes;

/** @class WorldModelPlugin plugins/worldmodel/worldmodel_plugin.h
 * Simple worldmodel plugin.
 * This plugin facilitates the Fawkes worldmodel, which agglomerates data produced
 * by low level Fawkes plugins and provides a consistent view of the world to
 * modules on higher levels.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
WorldModelPlugin::WorldModelPlugin(Configuration *config)
  : Plugin(config)
{
  WorldModelNetworkThread* net_thread = new WorldModelNetworkThread();
  thread_list.push_back(new WorldModelThread(net_thread));
  thread_list.push_back(net_thread);
}

PLUGIN_DESCRIPTION("Aggregates data to provide central world model")
EXPORT_PLUGIN(WorldModelPlugin)
