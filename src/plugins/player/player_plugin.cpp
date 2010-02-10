
/***************************************************************************
 *  player_plugin.h - Fawkes Player Plugin
 *
 *  Created: Tue Aug 05 13:11:02 2008
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

#include <plugins/player/player_plugin.h>

#include "playerc_thread.h"
#include "f2p_thread.h"
#include "timesync_thread.h"
#include "postsync_thread.h"

using namespace fawkes;

/** @class PlayerPlugin <plugins/player/player_plugin.h>
 * Player plugin for Fawkes.
 * This plugin integrates Fawkes with Player, for example for accessing
 * a simulator.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
PlayerPlugin::PlayerPlugin(Configuration *config)
  : Plugin(config)
{
  PlayerClientThread *pct = new PlayerClientThread();
  thread_list.push_back(pct);
  thread_list.push_back(new PlayerF2PThread(pct));
  thread_list.push_back(new PlayerTimeSyncThread());
  thread_list.push_back(new PlayerPostSyncThread());
}


EXPORT_PLUGIN(PlayerPlugin)
PLUGIN_DESCRIPTION("Player framework adapter")

