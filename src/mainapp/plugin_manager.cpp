
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
#include <mainapp/thread_manager.h>
#include <core/threading/thread_initializer.h>
#include <core/plugin.h>
#include <utils/plugin/plugin_loader.h>
#include <utils/logging/liblogger.h>
#include <utils/logging/console.h>

#include <netcomm/fawkes/component_ids.h>
#include <netcomm/fawkes/hub.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/** @class FawkesPluginManager mainapp/plugin_manager.h
 * Fawkes Plugin Manager.
 * This class provides a manager for the plugins used in fawkes. It can
 * load and unload modules.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param thread_manager thread manager plugin threads will be added to
 * and removed from appropriately.
 */
FawkesPluginManager::FawkesPluginManager(FawkesThreadManager *thread_manager)
  : FawkesNetworkHandler(FAWKES_CID_PLUGINMANAGER)
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
  for (pit = plugins.begin(); pit != plugins.end(); ++pit) {
    thread_manager->force_remove((*pit).second->threads());
    plugin_loader->unload( (*pit).second );
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
      e.printTrace();
      LibLogger::log_error("FawkesPluginManager", "Could not initialize one or more threads of plugin %s, unloading plugin", plugin_type);
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
FawkesPluginManager::process_after_loop()
{
  inbound_queue.lock();

  while ( ! inbound_queue.empty() ) {
    FawkesNetworkMessage *msg = inbound_queue.front();

    switch (msg->msgid()) {
    case MSG_PLUGIN_LOAD:
      if ( msg->payload_size() != sizeof(plugin_load_msg_t) ) {
	printf("Invalid load message size\n");
      } else {
	plugin_load_msg_t *m = (plugin_load_msg_t *)msg->payload();
	char name[PLUGIN_MSG_NAME_LENGTH + 1];
	name[PLUGIN_MSG_NAME_LENGTH] = 0;
	strncpy(name, m->name, PLUGIN_MSG_NAME_LENGTH);
	try {
	  LibLogger::log_info("FawkesPluginManager", "Trying to load %s", name);

	  load(name);
	  plugin_loaded_msg_t *r = (plugin_loaded_msg_t *)calloc(1, sizeof(plugin_loaded_msg_t));
	  strncpy(r->name, name, PLUGIN_MSG_NAME_LENGTH);
	  r->plugin_id = plugin_ids[name];
	  hub->broadcast(FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOADED, r, sizeof(plugin_loaded_msg_t));
	} catch (Exception &e) {
	  plugin_load_failed_msg_t *r = (plugin_load_failed_msg_t *)calloc(1, sizeof(plugin_load_failed_msg_t));
	  strncpy(r->name, name, PLUGIN_MSG_NAME_LENGTH);
	  hub->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOAD_FAILED,
		    r, sizeof(plugin_load_failed_msg_t));
	  LibLogger::log_warn("FawkesPluginManager", "Plugin %s could not be loaded", name);
	  LibLogger::log_warn("FawkesPluginManager", e);
	}
      }
      break;

    case MSG_PLUGIN_UNLOAD:
      if ( msg->payload_size() != sizeof(plugin_unload_msg_t) ) {
	printf("Invalid unload message size\n");
      } else {
	plugin_unload_msg_t *m = (plugin_unload_msg_t *)msg->payload();
	char name[PLUGIN_MSG_NAME_LENGTH + 1];
	name[PLUGIN_MSG_NAME_LENGTH] = 0;
	strncpy(name, m->name, PLUGIN_MSG_NAME_LENGTH);
	try {
	  LibLogger::log_debug("FawkesPluginManager", "Unloading %s", name);
	  unload(name);
	  plugin_unloaded_msg_t *r = (plugin_unloaded_msg_t *)calloc(1, sizeof(plugin_unloaded_msg_t));
	  strncpy(r->name, name, PLUGIN_MSG_NAME_LENGTH);
	  hub->broadcast(FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNLOADED,
			 r, sizeof(plugin_unloaded_msg_t));
	} catch (Exception &e) {
	  plugin_unload_failed_msg_t *r = (plugin_unload_failed_msg_t *)calloc(1, sizeof(plugin_unload_failed_msg_t));
	  strncpy(r->name, name, PLUGIN_MSG_NAME_LENGTH);
	  hub->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNLOAD_FAILED,
		    r, sizeof(plugin_unload_failed_msg_t));
	  LibLogger::log_warn("FawkesPluginManager", "Plugin %s could not be unloaded", name);
	  LibLogger::log_warn("FawkesPluginManager", e);
	}
      }
      break;

    default:
      // error
      break;
    }

    msg->unref();
    inbound_queue.pop();
  }

  inbound_queue.unlock();
}


void
FawkesPluginManager::handle_network_message(FawkesNetworkMessage *msg)
{
  msg->ref();
  inbound_queue.push_locked(msg);
}


void
FawkesPluginManager::client_connected(unsigned int clid)
{
  // send out messages with all loaded plugins
  plugins_mutex->lock();
  if ( plugins.size() == 0 ) {
    hub->send(clid, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_NONE_LOADED);
  } else {
    for (pit = plugins.begin(); pit != plugins.end(); ++pit) {
      plugin_loaded_msg_t *r = (plugin_loaded_msg_t *)calloc(1, sizeof(plugin_loaded_msg_t));
      strncpy(r->name, (*pit).first.c_str(), PLUGIN_MSG_NAME_LENGTH);
      r->plugin_id = plugin_ids[(*pit).first];
      hub->send(clid, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOADED,
		r, sizeof(plugin_loaded_msg_t));
    }
  }
  plugins_mutex->unlock();
}


void
FawkesPluginManager::client_disconnected(unsigned int clid)
{
}
