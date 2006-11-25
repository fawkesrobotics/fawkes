
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
#include <core/threading/thread_manager.h>
#include <core/plugin.h>
#include <utils/plugin/plugin_loader.h>

#include <netcomm/fawkes/component_ids.h>

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
FawkesPluginManager::FawkesPluginManager(ThreadManager *thread_manager)
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
    thread_manager->remove((*pit).second->threads());
    plugin_loader->unload( (*pit).second );
  }
  plugins.clear();
  plugin_ids.clear();
  delete plugin_loader;
  delete plugins_mutex;
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
    plugins[plugin_type] = plugin;
    plugin_ids[plugin_type] = next_plugin_id++;
    thread_manager->add(plugin->threads());
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
  thread_manager->remove(plugins[plugin_type]->threads());
  plugin_loader->unload(plugins[plugin_type]);
  plugins.erase(plugin_type);
  plugin_ids.erase(plugin_type);
  plugins_mutex->unlock();
}


/** Load all plugins currently in load queue.
 */
void
FawkesPluginManager::load()
{
  load_queue.lock();
  while (! load_queue.empty() ) {
    std::pair<unsigned int, std::string> &p = load_queue.front();
    try {
      load(p.second.c_str());
      plugin_loaded_msg_t *r = (plugin_loaded_msg_t *)calloc(1, sizeof(plugin_loaded_msg_t));
      strncpy(r->name, p.second.c_str(), PLUGIN_MSG_NAME_LENGTH);
      r->plugin_id = plugin_ids[p.second];
      send(p.first, MSG_PLUGIN_LOADED, r, sizeof(plugin_loaded_msg_t));
    } catch (PluginNotFoundException &e) {
      printf("Plugin %s could not be found\n", p.second.c_str());
    }
    load_queue.pop();
  }
  load_queue.unlock();
}


void
FawkesPluginManager::handleNetworkMessage(FawkesNetworkMessage *msg)
{
  switch (msg->msgid()) {
  case MSG_PLUGIN_LOAD:
    if ( msg->payload_size() != sizeof(plugin_load_msg_t) ) {
      printf("Invalid message size\n");
    } else {
      std::pair<unsigned int, std::string> p;
      plugin_load_msg_t *m = (plugin_load_msg_t *)msg->payload();
      p.first = msg->clid();
      char name[PLUGIN_MSG_NAME_LENGTH + 1];
      name[PLUGIN_MSG_NAME_LENGTH] = 0;
      strncpy(name, m->name, PLUGIN_MSG_NAME_LENGTH);
      p.second = name;
      load_queue.push_locked(p);
    }
    break;

  default:
    // error
    break;
  }
}


void
FawkesPluginManager::clientConnected(unsigned int clid)
{
  // send out messages with all loaded plugins
  plugins_mutex->lock();
  for (pit = plugins.begin(); pit != plugins.end(); ++pit) {
    plugin_loaded_msg_t *r = (plugin_loaded_msg_t *)calloc(1, sizeof(plugin_loaded_msg_t));
    strncpy(r->name, (*pit).first.c_str(), PLUGIN_MSG_NAME_LENGTH);
    r->plugin_id = plugin_ids[(*pit).first];
    send(clid, MSG_PLUGIN_LOADED, r, sizeof(plugin_loaded_msg_t));
  }
  plugins_mutex->unlock();
}


void
FawkesPluginManager::clientDisconnected(unsigned int clid)
{
}
