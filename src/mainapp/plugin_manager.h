
/***************************************************************************
 *  plugin_manager.h - Fawkes plugin manager
 *
 *  Created: Wed Nov 15 23:28:01 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __FAWKES_PLUGIN_MANAGER_H_
#define __FAWKES_PLUGIN_MANAGER_H_

#include <netcomm/fawkes/handler.h>
#include <core/utils/lock_queue.h>

#include <map>
#include <string>
#include <utility>

class FawkesThreadManager;
class Plugin;
class PluginLoader;
class Mutex;

class FawkesPluginManager : public FawkesNetworkHandler
{
 public:
  FawkesPluginManager(FawkesThreadManager *thread_manager);
  ~FawkesPluginManager();

  void load(const char *plugin_type);
  void unload(const char *plugin_type);

  virtual void handleNetworkMessage(FawkesNetworkMessage *msg);
  virtual void clientConnected(unsigned int clid);
  virtual void clientDisconnected(unsigned int clid);
  virtual void processAfterLoop();

 private:
  FawkesThreadManager  *thread_manager;
  PluginLoader         *plugin_loader;

  Mutex *plugins_mutex;

  std::map< std::string, Plugin * > plugins;
  std::map< std::string, Plugin * >::iterator pit;

  unsigned int next_plugin_id;
  std::map< std::string, unsigned int > plugin_ids;

  LockQueue< FawkesNetworkMessage * > inbound_queue;
};

#endif
