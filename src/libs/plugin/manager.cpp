
/***************************************************************************
 *  manager.cpp - Fawkes plugin manager
 *
 *  Generated: Wed Nov 15 23:31:55 2006 (on train to Cologne)
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

#include <plugin/manager.h>
#include <plugin/net/messages.h>
#include <plugin/net/list_message.h>
#include <plugin/loader.h>

#include <core/plugin.h>
#include <core/threading/thread_collector.h>
#include <core/threading/thread_initializer.h>
#include <utils/logging/liblogger.h>
#include <utils/system/fam_thread.h>
#include <config/config.h>

#include <netcomm/fawkes/component_ids.h>
#include <netcomm/fawkes/hub.h>

#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <cerrno>

#include <sys/types.h>
#include <dirent.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class PluginManager <plugin/manager.h>
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
 * @param thread_collector thread manager plugin threads will be added to
 * and removed from appropriately.
 * @param config Fawkes configuration
 * @param meta_plugin_prefix Path prefix for meta plugins
 */
PluginManager::PluginManager(ThreadCollector *thread_collector,
			     Configuration *config,
			     const char *meta_plugin_prefix)
  : Thread("PluginManager", Thread::OPMODE_WAITFORWAKEUP),
    FawkesNetworkHandler(FAWKES_CID_PLUGINMANAGER),
    ConfigurationChangeHandler(meta_plugin_prefix)
{
  plugins.clear();
  this->thread_collector = thread_collector;
  plugin_loader = new PluginLoader(PLUGINDIR, config);
  next_plugin_id = 1;
  __config = config;
  __meta_plugin_prefix = meta_plugin_prefix;

  init_pinfo_cache();

  __config->add_change_handler(this);

  __fam_thread = new FamThread();
  RefPtr<FileAlterationMonitor> fam = __fam_thread->get_fam();
  fam->add_filter("^[^.].*\\.so$"); 
  fam->add_listener(this);
  fam->watch_dir(PLUGINDIR);
  __fam_thread->start();
}


/** Destructor. */
PluginManager::~PluginManager()
{
  __fam_thread->cancel();
  __fam_thread->join();
  delete __fam_thread;
  __config->rem_change_handler(this);
  __pinfo_cache.lock();
  __pinfo_cache.clear();
  __pinfo_cache.unlock();
  // Unload all plugins
  for (rpit = plugins.rbegin(); rpit != plugins.rend(); ++rpit) {
    thread_collector->force_remove((*rpit).second->threads());
    plugin_loader->unload( (*rpit).second );
  }
  plugins.clear();
  plugin_ids.clear();
  delete plugin_loader;
}


/** Set Fawkes network hub.
 * The hub will be used for network communication. The PluginManager
 * is automatically added as handler to the hub for plugin messages.
 * @param hub Fawkes network hub
 */
void
PluginManager::set_hub(FawkesNetworkHub *hub)
{
  this->hub = hub;
  hub->add_handler( this );
}


void
PluginManager::init_pinfo_cache()
{
  __pinfo_cache.lock();

  Thread::CancelState old_state;
  Thread::set_cancel_state(Thread::CANCEL_DISABLED);

  DIR *plugin_dir;
  struct dirent* dirp;
  /* constant for this somewhere? */
  const char *file_ext = ".so";

  if ( NULL == (plugin_dir = opendir(PLUGINDIR)) ) {
    throw Exception(errno, "Plugin directory %s could not be opened", plugin_dir);
  }

  for (unsigned int i = 0; NULL != (dirp = readdir(plugin_dir)); ++i) {
    char *file_name   = dirp->d_name;
    char *pos         = strstr(file_name, file_ext);
    std::string plugin_name = std::string(file_name).substr(0, strlen(file_name) - strlen(file_ext));
    if (NULL != pos) {
      try {
	__pinfo_cache.push_back(make_pair(plugin_name,
					  plugin_loader->get_description(plugin_name.c_str())));
      } catch (Exception &e) {
	LibLogger::log_warn("PluginManager", "Could not get description of plugin %s, "
			    "exception follows", plugin_name.c_str());
	LibLogger::log_warn("PluginManager", e);
      }
    }
  }

  closedir(plugin_dir);

  try {
    Configuration::ValueIterator *i = __config->search(__meta_plugin_prefix.c_str());
    while (i->next()) {
      if (i->is_string()) {
	std::string p = std::string(i->path()).substr(__meta_plugin_prefix.length());
	std::string s = std::string("Meta: ") + i->get_string();
	
	__pinfo_cache.push_back(make_pair(p, s));
      }
    }
    delete i;
  } catch (Exception &e) {
  }

  __pinfo_cache.sort();
  __pinfo_cache.unlock();

  Thread::set_cancel_state(old_state);
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
PluginManager::list_avail()
{
  PluginListMessage *m = new PluginListMessage();

  std::list<std::pair<std::string, std::string> >::iterator i;
  for (i = __pinfo_cache.begin(); i != __pinfo_cache.end(); ++i) {
    m->append(i->first.c_str(), i->first.length());
    m->append(i->second.c_str(), i->second.length());
  }
  return m;
}

PluginListMessage *
PluginManager::list_loaded()
{
  PluginListMessage *m = new PluginListMessage();

  plugins.lock();
  for (pit = plugins.begin(); pit != plugins.end(); ++pit) {
    m->append((*pit).first.c_str(), (*pit).first.length());
  }
  for (__mpit = __meta_plugins.begin(); __mpit != __meta_plugins.end(); ++__mpit) {
    m->append(__mpit->first.c_str(), __mpit->first.length());
  }
  plugins.unlock();

  return m;
}


void
PluginManager::send_load_failure(const char *plugin_name,
				       unsigned int client_id)
{
  plugin_load_failed_msg_t *r = (plugin_load_failed_msg_t *)calloc(1, sizeof(plugin_load_failed_msg_t));
  strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
  hub->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOAD_FAILED,
	    r, sizeof(plugin_load_failed_msg_t));
}


void
PluginManager::send_load_success(const char *plugin_name, unsigned int client_id)
{
  plugin_loaded_msg_t *r = (plugin_loaded_msg_t *)calloc(1, sizeof(plugin_loaded_msg_t));
  strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
  r->plugin_id = plugin_ids[plugin_name];
  hub->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOADED,
	    r, sizeof(plugin_loaded_msg_t));
}


void
PluginManager::send_unloaded(const char *plugin_name)
{
  __subscribers.lock();
  for (__ssit = __subscribers.begin(); __ssit != __subscribers.end(); ++__ssit) {
    send_unload_success(plugin_name, *__ssit);
  }
  __subscribers.unlock();
}


void
PluginManager::send_loaded(const char *plugin_name)
{
  __subscribers.lock();
  for (__ssit = __subscribers.begin(); __ssit != __subscribers.end(); ++__ssit) {
    send_load_success(plugin_name, *__ssit);
  }
  __subscribers.unlock();
}


void
PluginManager::send_unload_failure(const char *plugin_name,
					 unsigned int client_id)
{
  plugin_unload_failed_msg_t *r = (plugin_unload_failed_msg_t *)calloc(1, sizeof(plugin_unload_failed_msg_t));
  strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
  hub->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNLOAD_FAILED,
	    r, sizeof(plugin_unload_failed_msg_t));
}


void
PluginManager::send_unload_success(const char *plugin_name, unsigned int client_id)
{
  plugin_unloaded_msg_t *r = (plugin_unloaded_msg_t *)calloc(1, sizeof(plugin_unloaded_msg_t));
  strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
  hub->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNLOADED,
	    r, sizeof(plugin_unloaded_msg_t));
}


/** Parse a list of plugin types.
 * Takes a comma-separated list of plugins and parses them into the individual
 * plugin names.
 * @param plugin_type_list string containing a comma-separated list of plugin types
 * @return parsed list of plugin types
 */
std::list<std::string>
PluginManager::parse_plugin_list(const char *plugin_list)
{
  std::list<std::string> rv;

  char *plugins = strdup(plugin_list);
  char *saveptr;
  char *plugin;

  plugin = strtok_r(plugins, ",", &saveptr);
  while ( plugin ) {
    rv.push_back(plugin);
    plugin = strtok_r(NULL, ",", &saveptr);
  }
  free(plugins);

  return rv;
}


/** Load plugin.
 * The loading is interrupted if any of the plugins does not load properly.
 * The already loaded plugins are *not* unloaded, but kept.
 * @param plugin_list string containing a comma-separated list of plugins
 * to load. The plugin list can contain meta plugins.
 */
void
PluginManager::load(const char *plugin_list)
{
  std::list<std::string> pp = parse_plugin_list(plugin_list);

  for (std::list<std::string>::iterator i = pp.begin(); i != pp.end(); ++i) {
    if ( i->length() == 0 ) continue;

    bool try_real_plugin = true;
    if ( __meta_plugins.find(*i) == __meta_plugins.end() ) {
      std::string meta_plugin = __meta_plugin_prefix + *i;
      try {
	std::string pset = __config->get_string(meta_plugin.c_str());
	if (pset.length() > 0) {
	  //printf("Going to load meta plugin %s (%s)\n", i->c_str(), pset.c_str());
	  __meta_plugins.lock();
	  // Setting has to happen here, so that a meta plugin will not cause an endless
	  // loop if it references itself!
	  __meta_plugins[*i] = pset;
	  try {
	    LibLogger::log_info("PluginManager", "Loading plugins %s for meta plugin %s",
				pset.c_str(), i->c_str());
	    load(pset.c_str());
	    send_loaded(i->c_str());
	  } catch (Exception &e) {
	    e.append("Could not initialize meta plugin %s, aborting loading.", i->c_str());
	    __meta_plugins.erase(*i);
	    __meta_plugins.unlock();
	    throw;
	  }
	  __meta_plugins.unlock();
	}
	try_real_plugin = false;
      } catch (ConfigEntryNotFoundException &e) {
	// no meta plugin defined by that name
	//printf("No meta plugin defined with the name %s\n", i->c_str());
	try_real_plugin = true;
      }
    }

    if (try_real_plugin && (plugins.find(*i) == plugins.end()) ) {
      try {
	//printf("Going to load real plugin %s\n", i->c_str());
	Plugin *plugin = plugin_loader->load(i->c_str());
	plugins.lock();
	try {
	  thread_collector->add(plugin->threads());
	  plugins[*i] = plugin;
	  plugin_ids[*i] = next_plugin_id++;
	  send_loaded(i->c_str());
	} catch (CannotInitializeThreadException &e) {
	  e.prepend("Plugin >>> %s <<< could not be initialized, unloading", i->c_str());
	  plugins.unlock();
	  plugin_loader->unload(plugin);
	  throw;
	}
	plugins.unlock();
      } catch (Exception &e) {
	if ( __meta_plugins.find(*i) == __meta_plugins.end() ) {
	  // only throw exception if no meta plugin with that name has already been loaded
	  throw;
	}
      }
    }
  }
}


/** Load plugin.
 * The loading is interrupted if any of the plugins does not load properly.
 * The already loaded plugins are *not* unloaded, but kept.
 * @param plugin_list string containing a comma-separated list of plugins
 * to load. The plugin list can contain meta plugins.
 * @param clid Fawkes network client ID of client that gets a success message
 * with the exact string that was put into
 */
void
PluginManager::load(const char *plugin_list, unsigned int clid)
{
  try {
    load(plugin_list);
    send_load_success(plugin_list, clid);
  } catch (Exception &e) {
    throw;
  }
}


/** Unload plugin.
 * Note that this method does not allow to pass a list of plugins, but it will
 * only accept a single plugin at a time.
 * @param plugin_name plugin to unload, can be a meta plugin.
 */
void
PluginManager::unload(const char *plugin_name)
{
  if ( plugins.find(plugin_name) != plugins.end() ) {
    plugins.lock();
    try {
      thread_collector->remove(plugins[plugin_name]->threads());
      plugin_loader->unload(plugins[plugin_name]);
      plugins.erase(plugin_name);
      plugin_ids.erase(plugin_name);
      send_unloaded(plugin_name);
      // find all meta plugins that required this module, this can no longer
      // be considered loaded
      __meta_plugins.lock();
      __mpit = __meta_plugins.begin();
      while (__mpit != __meta_plugins.end()) {
	std::list<std::string> pp = parse_plugin_list(__mpit->second.c_str());

	bool erase = false;
	for (std::list<std::string>::iterator i = pp.begin(); i != pp.end(); ++i) {
	  if ( *i == plugin_name ) {
	    erase = true;
	    break;
	  }
	}
	if ( erase ) {
	  LockMap< std::string, std::string >::iterator tmp = __mpit;
	  ++__mpit;
	  send_unloaded(tmp->first.c_str());
	  __meta_plugins.erase(tmp);
	} else {
	  ++__mpit;
	}
      }
      __meta_plugins.unlock();
      
    } catch (Exception &e) {
      LibLogger::log_error("PluginManager", "Could not finalize one or more threads of plugin %s, NOT unloading plugin", plugin_name);
      plugins.unlock();
      throw;
    }
    plugins.unlock();
  } else if (__meta_plugins.find(plugin_name) != __meta_plugins.end()) {
    std::list<std::string> pp = parse_plugin_list(__meta_plugins[plugin_name].c_str());

    for (std::list<std::string>::reverse_iterator i = pp.rbegin(); i != pp.rend(); ++i) {
      if ( i->length() == 0 ) continue;
      if ( (plugins.find(*i) == plugins.end()) &&
	   (__meta_plugins.find(*i) != __meta_plugins.end()) ) {
	continue;
      }
      __meta_plugins.lock();
      __meta_plugins.erase(*i);
      __meta_plugins.unlock();
      LibLogger::log_info("PluginManager", "UNloading plugin %s for meta plugin %s",
			  i->c_str(), plugin_name);
      unload(i->c_str());
    }
  }
}


/** Unload plugin.
 * Note that this method does not allow to pass a list of plugins, but it will
 * only accept a single plugin at a time.
 * @param plugin_name plugin to unload, can be a meta plugin.
 * @param clid Fawkes network client ID of client that gets a success message
 * with the exact string that was put into
 */
void
PluginManager::unload(const char *plugin_name, unsigned int clid)
{
  try {
    unload(plugin_name);
    send_unload_success(plugin_name, clid);
  } catch (Exception &e) {
    throw;
  }
}


/** Process all network messages that have been received.
 */
void
PluginManager::loop()
{
  while ( ! inbound_queue.empty() ) {
    FawkesNetworkMessage *msg = inbound_queue.front();

    switch (msg->msgid()) {
    case MSG_PLUGIN_LOAD:
      if ( msg->payload_size() != sizeof(plugin_load_msg_t) ) {
	LibLogger::log_error("PluginManager", "Invalid load message size");
      } else {
	plugin_load_msg_t *m = (plugin_load_msg_t *)msg->payload();
	char name[PLUGIN_MSG_NAME_LENGTH + 1];
	name[PLUGIN_MSG_NAME_LENGTH] = 0;
	strncpy(name, m->name, PLUGIN_MSG_NAME_LENGTH);

	if ( plugin_loader->is_loaded(name) ) {
	  LibLogger::log_info("PluginManager", "Client requested loading of %s which is already loaded", name);
	  send_load_success(name, msg->clid());
	} else {
	  LibLogger::log_info("PluginManager", "Loading plugin %s", name);
	  try {
	    load(name, msg->clid());
	  } catch (Exception &e) {
	    LibLogger::log_error("PluginManager", "Failed to load plugin %s", name);
	    LibLogger::log_error("PluginManager", e);
	    send_load_failure(name, msg->clid());
	  }
	}
      }
      break;

    case MSG_PLUGIN_UNLOAD:
      if ( msg->payload_size() != sizeof(plugin_unload_msg_t) ) {
	LibLogger::log_error("PluginManager", "Invalid unload message size.");
      } else {
	plugin_unload_msg_t *m = (plugin_unload_msg_t *)msg->payload();
	char name[PLUGIN_MSG_NAME_LENGTH + 1];
	name[PLUGIN_MSG_NAME_LENGTH] = 0;
	strncpy(name, m->name, PLUGIN_MSG_NAME_LENGTH);

	if ( (plugins.find(name) == plugins.end()) &&
	     (__meta_plugins.find(name) == __meta_plugins.end()) ) {
	  LibLogger::log_info("PluginManager", "Client requested unloading of %s which is not loaded", name);
	  send_unload_success(name, msg->clid());
	} else {
	  LibLogger::log_info("PluginManager", "UNloading plugin %s", name);
	  try {
	    unload(name, msg->clid());
	  } catch (Exception &e) {
	    LibLogger::log_error("PluginManager", "Failed to unload plugin %s", name);
	    LibLogger::log_error("PluginManager", e);
	    send_unload_failure(name, msg->clid());
	  }
	}
      }
      break;

    case MSG_PLUGIN_LIST_AVAIL:
      try {
	LibLogger::log_debug("PluginManager", "Sending list of all available plugins");
	PluginListMessage *plm = list_avail();
	hub->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_AVAIL_LIST, plm);
      } catch (Exception &e) {
	hub->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_AVAIL_LIST_FAILED);
      }
      break;

    case MSG_PLUGIN_LIST_LOADED:
      try {
	LibLogger::log_debug("PluginManager", "Sending list of all loaded plugins");
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
PluginManager::handle_network_message(FawkesNetworkMessage *msg)
{
  msg->ref();
  inbound_queue.push_locked(msg);
  wakeup();
}


void
PluginManager::client_connected(unsigned int clid)
{
}


void
PluginManager::client_disconnected(unsigned int clid)
{
  __subscribers.remove_locked(clid);
}


void
PluginManager::config_tag_changed(const char *new_tag)
{
}

void
PluginManager::config_value_changed(const char *path, int value)
{
  LibLogger::log_warn("PluginManager", "Integer value changed in meta plugins "
		      "path prefix at %s, ignoring", path);
}

void
PluginManager::config_value_changed(const char *path, unsigned int value)
{
  LibLogger::log_warn("PluginManager", "Unsigned integer value changed in meta "
		      "plugins path prefix at %s, ignoring", path);
}

void
PluginManager::config_value_changed(const char *path, float value)
{
  LibLogger::log_warn("PluginManager", "Float value changed in meta "
		      "plugins path prefix at %s, ignoring", path);
}

void
PluginManager::config_value_changed(const char *path, bool value)
{
  LibLogger::log_warn("PluginManager", "Boolean value changed in meta "
		      "plugins path prefix at %s, ignoring", path);
}

void
PluginManager::config_value_changed(const char *path, const char *value)
{
  __pinfo_cache.lock();
  std::string p = std::string(path).substr(__meta_plugin_prefix.length());
  std::string s = std::string("Meta: ") + value;
  std::list<std::pair<std::string, std::string> >::iterator i;
  bool found = false;
  for (i = __pinfo_cache.begin(); i != __pinfo_cache.end(); ++i) {
    if (p == i->first) {
      i->second = s;
      found = true;
      break;
    }
  }
  if (! found) {
    __pinfo_cache.push_back(make_pair(p, s));
  }
  __pinfo_cache.unlock();
}

void
PluginManager::config_value_erased(const char *path)
{
  __pinfo_cache.lock();
  std::string p = std::string(path).substr(__meta_plugin_prefix.length());
  std::list<std::pair<std::string, std::string> >::iterator i;
  for (i = __pinfo_cache.begin(); i != __pinfo_cache.end(); ++i) {
    if (p == i->first) {
      __pinfo_cache.erase(i);
      break;
    }
  }
  __pinfo_cache.unlock();
}


void
PluginManager::fam_event(const char *filename, unsigned int mask)
{
  /* constant for this somewhere? */
  const char *file_ext = ".so";

  char *pos               = strstr(filename, file_ext);
  std::string p = std::string(filename).substr(0, strlen(filename) - strlen(file_ext));
  if (NULL != pos) {
    __pinfo_cache.lock();
    bool found = false;
    std::list<std::pair<std::string, std::string> >::iterator i;
    for (i = __pinfo_cache.begin(); i != __pinfo_cache.end(); ++i) {
      if (p == i->first) {
	found = true;
	if ((mask & FAM_DELETE) || (mask & FAM_MOVED_FROM)) {
	  __pinfo_cache.erase(i);
	} else {
	  try {
	    i->second = plugin_loader->get_description(p.c_str());
	  } catch (Exception &e) {
	    LibLogger::log_warn("PluginManager", "Could not get possibly modified "
				"description of plugin %s, exception follows", 
				p.c_str());
	    LibLogger::log_warn("PluginManager", e);
	  }
	}
	break;
      }
    }
    if (! found &&
	!(mask & FAM_ISDIR) &&
	((mask & FAM_MODIFY) || (mask & FAM_MOVED_TO) || (mask & FAM_CREATE))) {
      if (plugin_loader->is_loaded(p.c_str())) {
	LibLogger::log_info("Plugin %s changed on disk, but is loaded, no new info "
			    "loaded", p.c_str());
      } else {
	try {
	  std::string s = plugin_loader->get_description(p.c_str());
	  __pinfo_cache.push_back(make_pair(p, s));
	} catch (Exception &e) {
	  LibLogger::log_warn("PluginManager", "Could not get possibly modified "
			      "description of plugin %s, exception follows", 
			      p.c_str());
	  LibLogger::log_warn("PluginManager", e);
	}
      }
    }
    __pinfo_cache.unlock();
  }
}


} // end namespace fawkes
