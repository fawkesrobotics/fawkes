
/***************************************************************************
 *  bbsync_plugin.cpp - Fawkes BlackBoardSynchronization Plugin
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

#include "bbsync_plugin.h"
#include "sync_thread.h"

#include <set>

using namespace fawkes;

/** @class BlackBoardSynchronizationPlugin "bbsync_plugin.h"
 * BlackBoard synchronization plugin.
 * This plugin synchronizes one or more (or even all) interfaces of two or
 * more Fawkes instances over the network.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
BlackBoardSynchronizationPlugin::BlackBoardSynchronizationPlugin(Configuration *config)
  : Plugin(config)
{
  std::set<std::string> peers;
  std::set<std::string> ignored_peers;

  std::string prefix = "/fawkes/bbsync/";
  std::string peers_prefix = prefix + "peers/";

  Configuration::ValueIterator *i = config->search(peers_prefix.c_str());
  while (i->next()) {
    std::string peer = std::string(i->path()).substr(peers_prefix.length());
    peer = peer.substr(0, peer.find("/"));

    if ( (peers.find(peer) == peers.end()) &&
	 (ignored_peers.find(peer) == ignored_peers.end()) ) {

      std::string peer_prefix = peers_prefix + peer + "/";

      bool active = true;
      try {
	active = config->get_bool((peer_prefix + "active").c_str());
      } catch (Exception &e) {} // ignored, assume enabled

      if (active) {
	//printf("Adding sync thread for peer %s\n", peer.c_str());
	BlackBoardSynchronizationThread *sync_thread;
	sync_thread = new BlackBoardSynchronizationThread(prefix, peer_prefix, peer);
	peers.insert(peer);
	thread_list.push_back(sync_thread);
      } else {
	//printf("Ignoring sync peer %s\n", peer.c_str());
	ignored_peers.insert(peer);
      }
    }
  }
  delete i;

  if ( thread_list.empty() ) {
    throw Exception("No synchronization peers configured, aborting");
  } else {
  }
}

PLUGIN_DESCRIPTION("Synchronize with remote Fawkes BlackBoards")
EXPORT_PLUGIN(BlackBoardSynchronizationPlugin)
