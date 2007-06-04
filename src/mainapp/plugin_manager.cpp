
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
#include <utils/logging/console.h>

#include <netcomm/fawkes/component_ids.h>
#include <netcomm/fawkes/hub.h>

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


void
FawkesPluginManager::request_load(const char *plugin_name, unsigned int client_id)
{
  std::string s = plugin_name;
  if ( load_requests.find(s) == load_requests.end() ) {
    try {
      LibLogger::log_debug("FawkesPluginManager", "Requesting loading of plugin '%s'", plugin_name);      
      plugin_loader->request_load(plugin_name);
    } catch (Exception &e) {
      LibLogger::log_error("FawkesPluginManager", e);
    }
  }
  load_requests[s].push_back(client_id);
  load_requests[s].sort();
  load_requests[s].unique();
}


void
FawkesPluginManager::request_unload(const char *plugin_name, unsigned int client_id)
{
  std::string s = plugin_name;
  if ( unload_requests.find(s) == unload_requests.end() ) {
    try {
      LibLogger::log_debug("FawkesPluginManager", "Requesting finalization of plugin '%s'", plugin_name);      
      thread_manager->remove_deferred(plugins[plugin_name]->threads());
    } catch (Exception &e) {
      LibLogger::log_error("FawkesPluginManager", e);
    }
  }
  unload_requests[s].push_back(client_id);
  unload_requests[s].sort();
  unload_requests[s].unique();
}


void
FawkesPluginManager::add_plugin_deferred(Plugin *plugin, const char *plugin_name)
{
  try {
    plugins_deferred[plugin_name] = plugin;
    plugin_ids_deferred[plugin_name] = next_plugin_id++;
    thread_manager->add_deferred(plugin->threads());
  } catch (CannotInitializeThreadException &e) {
    LibLogger::log_error("FawkesPluginManager", "Could not initialize one or more threads of plugin %s, unloading plugin", plugin_name);
    plugin_loader->unload(plugin);
    throw;
  }
}


void
FawkesPluginManager::send_load_failure(const char *plugin_name,
				       std::list<unsigned int> &clients)
{
  for (lrci = clients.begin(); lrci != clients.end(); ++lrci) {
    plugin_load_failed_msg_t *r = (plugin_load_failed_msg_t *)calloc(1, sizeof(plugin_load_failed_msg_t));
    strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
    hub->send((*lrci), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOAD_FAILED,
	      r, sizeof(plugin_load_failed_msg_t));
  }
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
FawkesPluginManager::send_unload_failure(const char *plugin_name,
					 std::list<unsigned int> &clients)
{
  for (lrci = clients.begin(); lrci != clients.end(); ++lrci) {
    plugin_unload_failed_msg_t *r = (plugin_unload_failed_msg_t *)calloc(1, sizeof(plugin_unload_failed_msg_t));
    strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
    hub->send((*lrci), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNLOAD_FAILED,
	      r, sizeof(plugin_unload_failed_msg_t));
  }
}

void
FawkesPluginManager::send_unload_success(const char *plugin_name, unsigned int client_id)
{
  plugin_unloaded_msg_t *r = (plugin_unloaded_msg_t *)calloc(1, sizeof(plugin_unloaded_msg_t));
  strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
  hub->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNLOADED,
	    r, sizeof(plugin_unloaded_msg_t));
}


void
FawkesPluginManager::check_loaded()
{
  plugins_mutex->lock();
  lri = load_requests.begin();
  while (lri != load_requests.end()) {
    std::string name = (*lri).first;
    if ( plugins_deferred.find(name) != plugins_deferred.end() ) {
      // Already loaded, deferred initialization running
      ++lri;
      continue;
    }
    if ( plugin_loader->finished_load(name.c_str()) ) {
      try {
	Plugin *p = plugin_loader->finish_deferred_load(name.c_str());

	// LibLogger::log_debug("FawkesPluginManager", "Adding plugin %s", name.c_str());
	add_plugin_deferred(p, name.c_str());

      } catch (Exception &e) {
	LibLogger::log_error("FawkesPluginManager", "Could not load plugin '%s'", name.c_str());
	LibLogger::log_error("FawkesPluginManager", e);
	send_load_failure(name.c_str(), (*lri).second);
      }
    }
    ++lri;
  }
  plugins_mutex->unlock();
}


void
FawkesPluginManager::check_finalized()
{
  plugins_mutex->lock();
  ulri = unload_requests.begin();
  while (ulri != unload_requests.end()) {
    std::string name = (*ulri).first;

    if ( plugins.find(name) == plugins.end() ) {
      // Not loaded
      ++ulri;
      unload_requests.erase(name.c_str());
      plugin_unloaded_msg_t *r = (plugin_unloaded_msg_t *)calloc(1, sizeof(plugin_unloaded_msg_t));
      strncpy(r->name, name.c_str(), PLUGIN_MSG_NAME_LENGTH);
      hub->broadcast(FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNLOADED, r, sizeof(plugin_unloaded_msg_t));
      continue;
    }

    try {
      if ( thread_manager->deferred_remove_done(plugins[name]->threads()) ) {
	plugin_loader->unload(plugins[name]);

	plugins.erase(name);
	plugin_ids.erase(name);
      
	plugin_unloaded_msg_t *r = (plugin_unloaded_msg_t *)calloc(1, sizeof(plugin_unloaded_msg_t));
	strncpy(r->name, name.c_str(), PLUGIN_MSG_NAME_LENGTH);
	hub->broadcast(FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNLOADED, r, sizeof(plugin_unloaded_msg_t));

	LibLogger::log_info("FawkesPluginManager", "Plugin '%s' finalized and unloaded successfully", name.c_str());

	++ulri;
	unload_requests.erase(name.c_str());
      } else {
	++ulri;
      }
    } catch (Exception &e) {
      LibLogger::log_error("FawkesPluginManager", "Could not unload plugin '%s'", name.c_str());
      LibLogger::log_error("FawkesPluginManager", e);
      send_unload_failure(name.c_str(), (*ulri).second);
      ++ulri;
      unload_requests.erase(name.c_str());
    }
  }
  plugins_mutex->unlock();
}


void
FawkesPluginManager::check_initialized()
{
  plugins_mutex->lock();
  pit = plugins_deferred.begin();
  while (pit != plugins_deferred.end()) {
    std::string name = (*pit).first;
    try {
      // LibLogger::log_debug("FawkesPluginManager", "Checking if plugin %s has finished initialization", name.c_str());
      if ( thread_manager->deferred_add_done((*pit).second->threads()) ) {
	plugins[name] = (*pit).second;
	plugin_ids[name] = plugin_ids_deferred[name];

	// LibLogger::log_debug("FawkesPluginManager", "Sending success message for %s", name.c_str());
	plugin_loaded_msg_t *r = (plugin_loaded_msg_t *)calloc(1, sizeof(plugin_loaded_msg_t));
	strncpy(r->name, name.c_str(), PLUGIN_MSG_NAME_LENGTH);
	r->plugin_id = plugin_ids[name];
	hub->broadcast(FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOADED, r, sizeof(plugin_loaded_msg_t));

	++pit;
	plugins_deferred.erase(name);
	plugin_ids_deferred.erase(name);

	LibLogger::log_info("FawkesPluginManager", "Plugin %s loaded and initialized successfully", name.c_str());

	load_requests.erase(name);

      } else {
	++pit;
      }
    } catch (Exception &e) {
      LibLogger::log_error("FawkesPluginManager", "Initialization of plugin failed");
      LibLogger::log_error("FawkesPluginManager", e);
      send_load_failure(name.c_str(), load_requests[name]);
      load_requests.erase(name);
      plugin_loader->unload((*pit).second);
      ++pit;
      plugins_deferred.erase(name);
      plugin_ids_deferred.erase(name);
    }
  }
  plugins_mutex->unlock();
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
  check_loaded();
  check_initialized();
  check_finalized();

  inbound_queue.lock();

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
	  LibLogger::log_info("FawkesPluginManager", "Requesting deferred loading of %ul %s", strlen(name), name);
	  request_load(name, msg->clid());
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
	  LibLogger::log_info("FawkesPluginManager", "Requesting deferred UNloading of %s", name);
	  request_unload(name, msg->clid());
	}
      }
      break;

    case MSG_PLUGIN_LIST_AVAIL:
      try {
	LibLogger::log_debug("FawkesPluginManager", "Sending list of all available plugins");
	PluginListMessage *plm = list_avail();
	hub->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LIST, plm);
      } catch (Exception &e) {
	hub->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LIST_AVAIL_FAILED);
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
