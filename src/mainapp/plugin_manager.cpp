
/***************************************************************************
 *  plugin_manager.cpp - Fawkes plugin manager
 *
 *  Generated: Wed Nov 15 23:31:55 2006 (on train to Cologne)
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

#include <mainapp/plugin_manager.h>
#include <mainapp/plugin_messages.h>
#include <mainapp/plugin_list_message.h>
#include <mainapp/thread_manager.h>
#include <core/threading/thread_initializer.h>
#include <core/plugin.h>
#include <utils/plugin/plugin_loader.h>
#include <utils/logging/liblogger.h>

#include <netcomm/fawkes/component_ids.h>
#include <netcomm/fawkes/hub.h>

#include <algorithm>
#include <cstring>
#include <cstdlib>

#include <sys/types.h>
#include <dirent.h>

/** @class FawkesPluginManager mainapp/plugin_manager.h
 * Fawkes Plugin Manager.
 * This class provides a manager for the plugins used in fawkes. It can
 * load and unload modules.
 *
 * @author Tim Niemueller
 */

/* IMPORANT IMPLEMENTER'S NOTE
 *
 * If you are going to work on this code mind the following: it is assumed
 * that only loop() will pop messages from the inbound queue. Thus the inbound
 * queue is only locked for this pop operation, not for the whole access time.
 * This is true as long as messages are only appended from the outside!
 * This is necessary to ensure that handle_network_message() will not hang
 * waiting for the queue lock.
 */

/** Constructor.
 * @param thread_manager thread manager plugin threads will be added to
 * and removed from appropriately.
 */
FawkesPluginManager::FawkesPluginManager(FawkesThreadManager *thread_manager)
  : Thread("FawkesPluginManager", Thread::OPMODE_WAITFORWAKEUP),
    FawkesNetworkHandler(FAWKES_CID_PLUGINMANAGER)
{
  plugins.clear();
  this->thread_manager = thread_manager;
  plugin_loader = new PluginLoader(PLUGINDIR);
  next_plugin_id = 1;
  plugins_mutex = new Mutex();
}


/** Destructor. */
FawkesPluginManager::~FawkesPluginManager()
{
  // Unload all plugins
  for (rpit = plugins.rbegin(); rpit != plugins.rend(); ++rpit) {
    thread_manager->force_remove((*rpit).second->threads());
    plugin_loader->unload( (*rpit).second );
  }
  plugins.clear();
  plugin_ids.clear();
  delete plugin_loader;
  delete plugins_mutex;
}


/** Set Fawkes network hub.
 * The hub will be used for network communication. The FawkesPluginManager
 * is automatically added as handler to the hub for plugin messages.
 * @param hub Fawkes network hub
 */
void
FawkesPluginManager::set_hub(FawkesNetworkHub *hub)
{
  this->hub = hub;
  hub->add_handler( this );
}

/** Generate list of all available plugins.
 * All files with the extension .so in the PLUGINDIR are returned.
 * @param num_plugins pointer to an unsigned int where the number
 * of all plugins is stored
 * @param plugin_list pointer to the string array where the list of 
 * all plugins is stored. Memory is allocated at this address and
 * has to be freed by the caller!
 */
PluginListMessage *
FawkesPluginManager::list_avail()
{
  DIR* plugin_dir;
  struct dirent* dirp;
  /* constant for this somewhere? */
  const char* file_ext = ".so";

  PluginListMessage *m = new PluginListMessage();

  if ( NULL == (plugin_dir = opendir(PLUGINDIR)) ) {
    LibLogger::log_error("FawkesPluginManager", "Opening Plugindir failed.");
    return m;
  }

  for (unsigned int i = 0; NULL != (dirp = readdir(plugin_dir)); ++i) {
    char* file_name = dirp->d_name;
    char* pos = strstr(file_name, file_ext);
    if (NULL != pos) {
      m->append(file_name, strlen(file_name) - strlen(file_ext));
    }
  }

  closedir(plugin_dir);

  return m;
}

PluginListMessage *
FawkesPluginManager::list_loaded()
{
  PluginListMessage *m = new PluginListMessage();

  plugins_mutex->lock();
  for (pit = plugins.begin(); pit != plugins.end(); ++pit) {
    m->append((*pit).first.c_str(), (*pit).first.length());
  }
  plugins_mutex->unlock();

  return m;
}


void
FawkesPluginManager::send_load_failure(const char *plugin_name,
				       unsigned int client_id)
{
  plugin_load_failed_msg_t *r = (plugin_load_failed_msg_t *)calloc(1, sizeof(plugin_load_failed_msg_t));
  strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
  hub->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOAD_FAILED,
	    r, sizeof(plugin_load_failed_msg_t));
}


void
FawkesPluginManager::send_load_success(const char *plugin_name, unsigned int client_id)
{
  plugin_loaded_msg_t *r = (plugin_loaded_msg_t *)calloc(1, sizeof(plugin_loaded_msg_t));
  strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
  r->plugin_id = plugin_ids[plugin_name];
  hub->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOADED,
	    r, sizeof(plugin_loaded_msg_t));
}


void
FawkesPluginManager::send_unloaded(const char *plugin_name)
{
  __subscribers.lock();
  for (__ssit = __subscribers.begin(); __ssit != __subscribers.end(); ++__ssit) {
    send_unload_success(plugin_name, *__ssit);
  }
  __subscribers.unlock();
}


void
FawkesPluginManager::send_loaded(const char *plugin_name)
{
  __subscribers.lock();
  for (__ssit = __subscribers.begin(); __ssit != __subscribers.end(); ++__ssit) {
    send_load_success(plugin_name, *__ssit);
  }
  __subscribers.unlock();
}


void
FawkesPluginManager::send_unload_failure(const char *plugin_name,
					 unsigned int client_id)
{
  plugin_unload_failed_msg_t *r = (plugin_unload_failed_msg_t *)calloc(1, sizeof(plugin_unload_failed_msg_t));
  strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
  hub->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNLOAD_FAILED,
	    r, sizeof(plugin_unload_failed_msg_t));
}


void
FawkesPluginManager::send_unload_success(const char *plugin_name, unsigned int client_id)
{
  plugin_unloaded_msg_t *r = (plugin_unloaded_msg_t *)calloc(1, sizeof(plugin_unloaded_msg_t));
  strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
  hub->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNLOADED,
	    r, sizeof(plugin_unloaded_msg_t));
}


/** Load plugin.
 * @param plugin_type plugin type to load
 */
void
FawkesPluginManager::load(const char *plugin_type)
{
  if ( plugins.find(plugin_type) != plugins.end() ) return;

  try {
    Plugin *plugin = plugin_loader->load(plugin_type);
    plugins_mutex->lock();
    try {
      thread_manager->add(plugin->threads());
      plugins[plugin_type] = plugin;
      plugin_ids[plugin_type] = next_plugin_id++;
    } catch (CannotInitializeThreadException &e) {
      e.append("Could not initialize one or more "
	       "threads of plugin %s, unloading plugin", plugin_type);
      plugins_mutex->unlock();
      plugin_loader->unload(plugin);
      throw;
    }
    plugins_mutex->unlock();
  } catch (Exception &e) {
    throw;
  }
}


/** Unload plugin.
 * @param plugin_type plugin type to unload.
 */
void
FawkesPluginManager::unload(const char *plugin_type)
{
  if ( plugins.find(plugin_type) == plugins.end() )  return;

  plugins_mutex->lock();
  try {
    thread_manager->remove(plugins[plugin_type]->threads());
    plugin_loader->unload(plugins[plugin_type]);
    plugins.erase(plugin_type);
    plugin_ids.erase(plugin_type);
  } catch (Exception &e) {
    LibLogger::log_error("FawkesPluginManager", "Could not finalize one or more threads of plugin %s, NOT unloading plugin", plugin_type);
    plugins_mutex->unlock();
    throw;
  }
  plugins_mutex->unlock();
}


/** Process all network messages that have been received.
 */
void
FawkesPluginManager::loop()
{
  while ( ! inbound_queue.empty() ) {
    FawkesNetworkMessage *msg = inbound_queue.front();

    switch (msg->msgid()) {
    case MSG_PLUGIN_LOAD:
      if ( msg->payload_size() != sizeof(plugin_load_msg_t) ) {
	LibLogger::log_error("FawkesPluginManager", "Invalid load message size");
      } else {
	plugin_load_msg_t *m = (plugin_load_msg_t *)msg->payload();
	char name[PLUGIN_MSG_NAME_LENGTH + 1];
	name[PLUGIN_MSG_NAME_LENGTH] = 0;
	strncpy(name, m->name, PLUGIN_MSG_NAME_LENGTH);

	if ( plugin_loader->is_loaded(name) ) {
	  LibLogger::log_info("FawkesPluginManager", "Client requested loading of %s which is already loaded", name);
	  send_load_success(name, msg->clid());
	} else {
	  LibLogger::log_info("FawkesPluginManager", "Loading plugin %s", name);
	  try {
	    load(name);
	    send_load_success(name, msg->clid());
	    send_loaded(name);
	  } catch (Exception &e) {
	    LibLogger::log_error("FawkesPluginManager", "Failed to load plugin %s", name);
	    LibLogger::log_error("FawkesPluginManager", e);
	    send_load_failure(name, msg->clid());
	  }
	}
      }
      break;

    case MSG_PLUGIN_UNLOAD:
      if ( msg->payload_size() != sizeof(plugin_unload_msg_t) ) {
	LibLogger::log_error("FawkesPluginManager", "Invalid unload message size.");
      } else {
	plugin_unload_msg_t *m = (plugin_unload_msg_t *)msg->payload();
	char name[PLUGIN_MSG_NAME_LENGTH + 1];
	name[PLUGIN_MSG_NAME_LENGTH] = 0;
	strncpy(name, m->name, PLUGIN_MSG_NAME_LENGTH);

	if ( ! plugin_loader->is_loaded(name) ) {
	  LibLogger::log_info("FawkesPluginManager", "Client requested unloading of %s which is not loaded", name);
	  send_unload_success(name, msg->clid());
	} else {
	  LibLogger::log_info("FawkesPluginManager", "UNloading plugin %s", name);
	  try {
	    unload(name);
	    send_unload_success(name, msg->clid());
	    send_unloaded(name);
	  } catch (Exception &e) {
	    LibLogger::log_error("FawkesPluginManager", "Failed to unload plugin %s", name);
	    LibLogger::log_error("FawkesPluginManager", e);
	    send_unload_failure(name, msg->clid());
	  }
	}
      }
      break;

    case MSG_PLUGIN_LIST_AVAIL:
      try {
	LibLogger::log_debug("FawkesPluginManager", "Sending list of all available plugins");
	PluginListMessage *plm = list_avail();
	hub->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_AVAIL_LIST, plm);
      } catch (Exception &e) {
	hub->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_AVAIL_LIST_FAILED);
      }
      break;

    case MSG_PLUGIN_LIST_LOADED:
      try {
	LibLogger::log_debug("FawkesPluginManager", "Sending list of all loaded plugins");
	PluginListMessage *plm = list_loaded();
	hub->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOADED_LIST, plm);
      } catch (Exception &e) {
	hub->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOADED_LIST_FAILED);
      }
      break;

    case MSG_PLUGIN_SUBSCRIBE_WATCH:
      __subscribers.lock();
      __subscribers.push_back(msg->clid());
      __subscribers.sort();
      __subscribers.unique();
      __subscribers.unlock();
      break;

    case MSG_PLUGIN_UNSUBSCRIBE_WATCH:
      __subscribers.remove_locked(msg->clid());
      break;

    default:
      // error
      break;
    }

    msg->unref();
    inbound_queue.pop_locked();
  }
}


void
FawkesPluginManager::handle_network_message(FawkesNetworkMessage *msg)
{
  msg->ref();
  inbound_queue.push_locked(msg);
  wakeup();
}


void
FawkesPluginManager::client_connected(unsigned int clid)
{
}


void
FawkesPluginManager::client_disconnected(unsigned int clid)
{
  __subscribers.remove_locked(clid);
}
