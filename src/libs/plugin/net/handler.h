
/***************************************************************************
 *  handler.h - Fawkes plugin network handler
 *
 *  Created: Thu Feb 12 10:23:02 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGIN_NET_HANDLER_H_
#define __PLUGIN_NET_HANDLER_H_

#include <netcomm/fawkes/handler.h>
#include <core/threading/thread.h>
#include <core/utils/lock_queue.h>
#include <core/utils/lock_list.h>
#include <core/utils/lock_map.h>
#include <config/change_handler.h>
#include <utils/system/fam.h>
#include <plugin/listener.h>

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
class PluginListMessage;
class Configuration;
class FamThread;

class PluginNetworkHandler
: public fawkes::Thread,
  public fawkes::FawkesNetworkHandler,
  public fawkes::PluginManagerListener
{
 public:
  PluginNetworkHandler(PluginManager *manager, FawkesNetworkHub *hub);
  ~PluginNetworkHandler();

  virtual void handle_network_message(FawkesNetworkMessage *msg);
  virtual void client_connected(unsigned int clid);
  virtual void client_disconnected(unsigned int clid);

  virtual void loop();

  virtual void plugin_loaded(const char *plugin_name);
  virtual void plugin_unloaded(const char *plugin_name);

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

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  PluginManager     *__manager;
  FawkesNetworkHub  *__hub;

  LockQueue< FawkesNetworkMessage * > __inbound_queue;

  LockList<unsigned int>           __subscribers;
  LockList<unsigned int>::iterator __ssit;
};

} // end namespace fawkes

#endif
