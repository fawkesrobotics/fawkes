
/***************************************************************************
 *  plugin_manager.h - Fawkes plugin manager
 *
 *  Created: Wed Nov 15 23:28:01 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __FAWKES_PLUGIN_MANAGER_H_
#define __FAWKES_PLUGIN_MANAGER_H_

#include <netcomm/fawkes/handler.h>
#include <core/threading/thread.h>
#include <core/utils/lock_queue.h>
#include <core/utils/lock_list.h>
#include <core/utils/lock_map.h>

#include <map>
#include <list>
#include <string>
#include <utility>

class FawkesThreadManager;
namespace fawkes {
  class FawkesNetworkHub;
  class Plugin;
  class PluginLoader;
  class Mutex;
  class PluginListMessage;
  class Configuration;
}

class FawkesPluginManager
: public fawkes::Thread,
  public fawkes::FawkesNetworkHandler
{
 public:
  FawkesPluginManager(FawkesThreadManager *thread_manager,
		      fawkes::Configuration *config,
		      const char *meta_plugin_prefix);
  ~FawkesPluginManager();

  void set_hub(fawkes::FawkesNetworkHub *hub);

  virtual void handle_network_message(fawkes::FawkesNetworkMessage *msg);
  virtual void client_connected(unsigned int clid);
  virtual void client_disconnected(unsigned int clid);

  virtual void loop();

  void load(const char *plugin_list);
  void unload(const char *plugin_name);

 private:
  fawkes::PluginListMessage * list_avail();
  fawkes::PluginListMessage * list_loaded();
  void send_load_failure(const char *plugin_name, unsigned int client_id);
  void send_load_success(const char *plugin_name, unsigned int client_id);
  void send_unload_failure(const char *plugin_name, unsigned int client_id);
  void send_unload_success(const char *plugin_name, unsigned int client_id);
  void send_loaded(const char *plugin_name);
  void send_unloaded(const char *plugin_name);

  void load(const char *plugin_list, unsigned int clid);
  void unload(const char *plugin_list, unsigned int clid);

  std::list<std::string>  parse_plugin_list(const char *plugin_type_list);

 private:
  FawkesThreadManager       *thread_manager;
  fawkes::PluginLoader      *plugin_loader;
  fawkes::FawkesNetworkHub  *hub;

  fawkes::LockMap< std::string, fawkes::Plugin * > plugins;
  fawkes::LockMap< std::string, fawkes::Plugin * >::iterator pit;
  fawkes::LockMap< std::string, fawkes::Plugin * >::reverse_iterator rpit;

  fawkes::LockMap< std::string, std::string > __meta_plugins;
  fawkes::LockMap< std::string, std::string >::iterator __mpit;

  unsigned int next_plugin_id;
  std::map< std::string, unsigned int > plugin_ids;

  fawkes::LockQueue< fawkes::FawkesNetworkMessage * > inbound_queue;

  fawkes::LockList<unsigned int>           __subscribers;
  fawkes::LockList<unsigned int>::iterator __ssit;

  fawkes::Configuration *__config;
  std::string __meta_plugin_prefix;
};

#endif
