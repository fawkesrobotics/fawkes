
/***************************************************************************
 *  manager.h - Fawkes plugin manager
 *
 *  Created: Wed Nov 15 23:28:01 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGIN_MANAGER_H_
#define __PLUGIN_MANAGER_H_

#include <netcomm/fawkes/handler.h>
#include <core/threading/thread.h>
#include <core/utils/lock_queue.h>
#include <core/utils/lock_list.h>
#include <core/utils/lock_map.h>
#include <config/change_handler.h>
#include <utils/system/fam.h>

#include <map>
#include <list>
#include <string>
#include <utility>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ThreadCollector;
class FawkesNetworkHub;
class Plugin;
class PluginLoader;
class Mutex;
class PluginListMessage;
class Configuration;
class FamThread;

class PluginManager
: public fawkes::Thread,
  public fawkes::FawkesNetworkHandler,
  public fawkes::ConfigurationChangeHandler,
  public FamListener
{
 public:
  PluginManager(ThreadCollector *thread_collector,
		      Configuration *config,
		      const char *meta_plugin_prefix);
  ~PluginManager();

  void set_hub(FawkesNetworkHub *hub);

  virtual void handle_network_message(FawkesNetworkMessage *msg);
  virtual void client_connected(unsigned int clid);
  virtual void client_disconnected(unsigned int clid);

  // for ConfigurationChangeHandler
  virtual void config_tag_changed(const char *new_tag);
  virtual void config_value_changed(const char *path, int value);
  virtual void config_value_changed(const char *path, unsigned int value);
  virtual void config_value_changed(const char *path, float value);
  virtual void config_value_changed(const char *path, bool value);
  virtual void config_value_changed(const char *path, const char *value);
  virtual void config_value_erased(const char *path);

  // for FamListener
  virtual void fam_event(const char *filename, unsigned int mask);

  virtual void loop();

  void load(const char *plugin_list);
  void unload(const char *plugin_name);

 private:
  PluginListMessage * list_avail();
  PluginListMessage * list_loaded();
  void send_load_failure(const char *plugin_name, unsigned int client_id);
  void send_load_success(const char *plugin_name, unsigned int client_id);
  void send_unload_failure(const char *plugin_name, unsigned int client_id);
  void send_unload_success(const char *plugin_name, unsigned int client_id);
  void send_loaded(const char *plugin_name);
  void send_unloaded(const char *plugin_name);

  void load(const char *plugin_list, unsigned int clid);
  void unload(const char *plugin_list, unsigned int clid);

  void init_pinfo_cache();

  std::list<std::string>  parse_plugin_list(const char *plugin_type_list);

 private:
  ThreadCollector   *thread_collector;
  PluginLoader      *plugin_loader;
  FawkesNetworkHub  *hub;

  LockMap< std::string, Plugin * > plugins;
  LockMap< std::string, Plugin * >::iterator pit;
  LockMap< std::string, Plugin * >::reverse_iterator rpit;

  LockMap< std::string, std::string > __meta_plugins;
  LockMap< std::string, std::string >::iterator __mpit;

  unsigned int next_plugin_id;
  std::map< std::string, unsigned int > plugin_ids;

  LockQueue< FawkesNetworkMessage * > inbound_queue;

  LockList<unsigned int>           __subscribers;
  LockList<unsigned int>::iterator __ssit;

  LockList<std::pair<std::string, std::string> > __pinfo_cache;

  Configuration *__config;
  std::string __meta_plugin_prefix;

  FamThread *__fam_thread;
};

} // end namespace fawkes

#endif
