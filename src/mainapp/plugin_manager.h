
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
#include <list>
#include <string>
#include <utility>

class FawkesThreadManager;
class FawkesNetworkHub;
class Plugin;
class PluginLoader;
class Mutex;
class PluginListMessage;

class FawkesPluginManager : public FawkesNetworkHandler
{
 public:
  FawkesPluginManager(FawkesThreadManager *thread_manager);
  ~FawkesPluginManager();

  void set_hub(FawkesNetworkHub *hub);

  virtual void handle_network_message(FawkesNetworkMessage *msg);
  virtual void client_connected(unsigned int clid);
  virtual void client_disconnected(unsigned int clid);
  virtual void process_after_loop();

 private:
  PluginListMessage * list_avail();
  void load(const char *plugin_type);
  void unload(const char *plugin_type);
  void request_load(const char *plugin_name, unsigned int client_id);
  void request_unload(const char *plugin_name, unsigned int client_id);
  void add_plugin(Plugin *plugin, const char *plugin_name);
  void send_load_failure(const char *plugin_name, std::list<unsigned int> &clients);
  void send_load_success(const char *plugin_name, unsigned int client_id);
  void check_loaded();
  void check_initialized();
  void send_unload_failure(const char *plugin_name, std::list<unsigned int> &clients);
  void send_unload_success(const char *plugin_name, unsigned int client_id);
  void check_finalized();
  void add_plugin_deferred(Plugin *plugin, const char *plugin_name);

 private:
  FawkesThreadManager  *thread_manager;
  PluginLoader         *plugin_loader;
  FawkesNetworkHub     *hub;

  Mutex *plugins_mutex;

  std::map< std::string, Plugin * > plugins;
  std::map< std::string, Plugin * > plugins_deferred;
  std::map< std::string, Plugin * >::iterator pit;

  unsigned int next_plugin_id;
  std::map< std::string, unsigned int > plugin_ids;
  std::map< std::string, unsigned int > plugin_ids_deferred;

  std::map< std::string, std::list< unsigned int > > load_requests;
  std::map< std::string, std::list< unsigned int > >::iterator lri;
  std::map< std::string, std::list< unsigned int > > unload_requests;
  std::map< std::string, std::list< unsigned int > >::iterator ulri;
  std::list< unsigned int >::iterator lrci;

  LockQueue< FawkesNetworkMessage * > inbound_queue;
};

#endif
