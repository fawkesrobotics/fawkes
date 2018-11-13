
/***************************************************************************
 *  handler.cpp - Fawkes plugin network handler
 *
 *  Created: Thu Feb 12 10:36:15 2009
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

#include <plugin/manager.h>
#include <plugin/net/handler.h>
#include <plugin/net/messages.h>
#include <plugin/net/list_message.h>

#include <logging/liblogger.h>

#include <netcomm/fawkes/component_ids.h>
#include <netcomm/fawkes/hub.h>

#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <cerrno>

namespace fawkes {

/** @class PluginNetworkHandler <plugin/net/handler.h>
 * Fawkes Plugin Network Handler.
 * This network handler handles requests of plugin lists and for loading/unloading
 * plugins received over the network.
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
 * @param manager plugin manager for the actual work
 * @param hub Fawkes network hub
 */
PluginNetworkHandler::PluginNetworkHandler(PluginManager *manager,
					   FawkesNetworkHub *hub)
  : Thread("PluginNetworkHandler", Thread::OPMODE_WAITFORWAKEUP),
    FawkesNetworkHandler(FAWKES_CID_PLUGINMANAGER)
{
  manager_ = manager;
  hub_ = hub;

  manager_->add_listener(this);
  hub_->add_handler(this);
}


/** Destructor. */
PluginNetworkHandler::~PluginNetworkHandler()
{
  hub_->remove_handler(this);
  manager_->remove_listener(this);
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
PluginNetworkHandler::list_avail()
{
  PluginListMessage *m = new PluginListMessage();

  std::list<std::pair<std::string, std::string> > available_plugins;
  available_plugins = manager_->get_available_plugins();

  std::list<std::pair<std::string, std::string> >::iterator i;
  for (i = available_plugins.begin(); i != available_plugins.end(); ++i) {
    m->append(i->first.c_str(), i->first.length());
    m->append(i->second.c_str(), i->second.length());
  }
  return m;
}

PluginListMessage *
PluginNetworkHandler::list_loaded()
{
  PluginListMessage *m = new PluginListMessage();

  std::list<std::string> loaded_plugins;
  loaded_plugins = manager_->get_loaded_plugins();

  std::list<std::string>::iterator i;
  for (i = loaded_plugins.begin(); i != loaded_plugins.end(); ++i) {
    m->append(i->c_str(), i->length());
  }

  return m;
}


void
PluginNetworkHandler::send_load_failure(const char *plugin_name,
				       unsigned int client_id)
{
  try {
    plugin_load_failed_msg_t *r = (plugin_load_failed_msg_t *)calloc(1, sizeof(plugin_load_failed_msg_t));
    strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH-1);
    hub_->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOAD_FAILED,
		r, sizeof(plugin_load_failed_msg_t));
  } catch (Exception &e) {
    LibLogger::log_warn("PluginNetworkHandler", "Failed to send load failure, exception follows");
    LibLogger::log_warn("PluginNetworkHandler", e);
  }
}


void
PluginNetworkHandler::send_load_success(const char *plugin_name, unsigned int client_id)
{
  try {
    plugin_loaded_msg_t *r = (plugin_loaded_msg_t *)calloc(1, sizeof(plugin_loaded_msg_t));
    strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH-1);
    hub_->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOADED,
		r, sizeof(plugin_loaded_msg_t));
  } catch (Exception &e) {
    LibLogger::log_warn("PluginNetworkHandler", "Failed to send load success, exception follows");
    LibLogger::log_warn("PluginNetworkHandler", e);
  }
}


void
PluginNetworkHandler::send_unloaded(const char *plugin_name)
{
  subscribers_.lock();
  try {
    for (ssit_ = subscribers_.begin(); ssit_ != subscribers_.end(); ++ssit_) {
      send_unload_success(plugin_name, *ssit_);
    }
  } catch (Exception &e) {
    LibLogger::log_warn("PluginNetworkHandler", "Failed to send unloaded, exception follows");
    LibLogger::log_warn("PluginNetworkHandler", e);
  }
  subscribers_.unlock();
}


void
PluginNetworkHandler::send_loaded(const char *plugin_name)
{
  subscribers_.lock();
  try {
    for (ssit_ = subscribers_.begin(); ssit_ != subscribers_.end(); ++ssit_) {
      send_load_success(plugin_name, *ssit_);
    }
  } catch (Exception &e) {
    LibLogger::log_warn("PluginNetworkHandler", "Failed to send loaded, exception follows");
    LibLogger::log_warn("PluginNetworkHandler", e);
  }
  subscribers_.unlock();
}


void
PluginNetworkHandler::send_unload_failure(const char *plugin_name,
					 unsigned int client_id)
{
  try {
    plugin_unload_failed_msg_t *r = (plugin_unload_failed_msg_t *)calloc(1, sizeof(plugin_unload_failed_msg_t));
    strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH-1);
    hub_->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNLOAD_FAILED,
		r, sizeof(plugin_unload_failed_msg_t));
  } catch (Exception &e) {
    LibLogger::log_warn("PluginNetworkHandler", "Failed to send unload failure, exception follows");
    LibLogger::log_warn("PluginNetworkHandler", e);
  }
}


void
PluginNetworkHandler::send_unload_success(const char *plugin_name, unsigned int client_id)
{
  try {
    plugin_unloaded_msg_t *r = (plugin_unloaded_msg_t *)calloc(1, sizeof(plugin_unloaded_msg_t));
    strncpy(r->name, plugin_name, PLUGIN_MSG_NAME_LENGTH-1);
    hub_->send(client_id, FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNLOADED,
		r, sizeof(plugin_unloaded_msg_t));
  } catch (Exception &e) {
    LibLogger::log_warn("PluginNetworkHandler", "Failed to send unload success, exception follows");
    LibLogger::log_warn("PluginNetworkHandler", e);
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
PluginNetworkHandler::load(const char *plugin_list, unsigned int clid)
{
  manager_->lock();
  try {
    manager_->load(plugin_list);
    send_load_success(plugin_list, clid);
  } catch (Exception &e) {
    LibLogger::log_error("PluginNetworkHandler", "Failed to load plugin %s", plugin_list);
    LibLogger::log_error("PluginNetworkHandler", e);
    send_load_failure(plugin_list, clid);
  }
  manager_->unlock();
}


/** Unload plugin.
 * Note that this method does not allow to pass a list of plugins, but it will
 * only accept a single plugin at a time.
 * @param plugin_name plugin to unload, can be a meta plugin.
 * @param clid Fawkes network client ID of client that gets a success message
 * with the exact string that was put into
 */
void
PluginNetworkHandler::unload(const char *plugin_name, unsigned int clid)
{
  manager_->lock();
  try {
    manager_->unload(plugin_name);
    send_unload_success(plugin_name, clid);
  } catch (Exception &e) {
    LibLogger::log_error("PluginNetworkHandler", "Failed to unload plugin %s", plugin_name);
    LibLogger::log_error("PluginNetworkHandler", e);
    send_unload_failure(plugin_name, clid);
  }
  manager_->unlock();
}


/** Process all network messages that have been received.
 */
void
PluginNetworkHandler::loop()
{
  while ( ! inbound_queue_.empty() ) {
    FawkesNetworkMessage *msg = inbound_queue_.front();

    switch (msg->msgid()) {
    case MSG_PLUGIN_LOAD:
      if ( msg->payload_size() != sizeof(plugin_load_msg_t) ) {
	LibLogger::log_error("PluginNetworkHandler", "Invalid load message size");
      } else {
	plugin_load_msg_t *m = (plugin_load_msg_t *)msg->payload();
	char name[PLUGIN_MSG_NAME_LENGTH + 1];
	name[PLUGIN_MSG_NAME_LENGTH] = 0;
	strncpy(name, m->name, PLUGIN_MSG_NAME_LENGTH);

	if ( manager_->is_loaded(name) ) {
	  LibLogger::log_info("PluginNetworkHandler", "Client requested loading of %s which is already loaded", name);
	  send_load_success(name, msg->clid());
	} else {
	  LibLogger::log_info("PluginNetworkHandler", "Loading plugin %s", name);
	  load(name, msg->clid());
	}
      }
      break;

    case MSG_PLUGIN_UNLOAD:
      if ( msg->payload_size() != sizeof(plugin_unload_msg_t) ) {
	LibLogger::log_error("PluginNetworkHandler", "Invalid unload message size.");
      } else {
	plugin_unload_msg_t *m = (plugin_unload_msg_t *)msg->payload();
	char name[PLUGIN_MSG_NAME_LENGTH + 1];
	name[PLUGIN_MSG_NAME_LENGTH] = 0;
	strncpy(name, m->name, PLUGIN_MSG_NAME_LENGTH);

	if ( !manager_->is_loaded(name) ) {
	  LibLogger::log_info("PluginNetworkHandler", "Client requested unloading of %s which is not loaded", name);
	  send_unload_success(name, msg->clid());
	} else {
	  LibLogger::log_info("PluginNetworkHandler", "UNloading plugin %s", name);
	  unload(name, msg->clid());
	}
      }
      break;

    case MSG_PLUGIN_LIST_AVAIL:
      try {
	      //LibLogger::log_debug("PluginNetworkHandler", "Sending list of all available plugins");
	PluginListMessage *plm = list_avail();
	hub_->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_AVAIL_LIST, plm);
      } catch (Exception &e) {
	hub_->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_AVAIL_LIST_FAILED);
      }
      break;

    case MSG_PLUGIN_LIST_LOADED:
      try {
	      //LibLogger::log_debug("PluginNetworkHandler", "Sending list of all loaded plugins");
	PluginListMessage *plm = list_loaded();
	hub_->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOADED_LIST, plm);
      } catch (Exception &e) {
	hub_->send(msg->clid(), FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_LOADED_LIST_FAILED);
      }
      break;

    case MSG_PLUGIN_SUBSCRIBE_WATCH:
      subscribers_.lock();
      subscribers_.push_back(msg->clid());
      subscribers_.sort();
      subscribers_.unique();
      subscribers_.unlock();
      break;

    case MSG_PLUGIN_UNSUBSCRIBE_WATCH:
      subscribers_.remove_locked(msg->clid());
      break;

    default:
      // error
      break;
    }

    msg->unref();
    inbound_queue_.pop_locked();
  }
}


void
PluginNetworkHandler::handle_network_message(FawkesNetworkMessage *msg)
{
  msg->ref();
  inbound_queue_.push_locked(msg);
  wakeup();
}


void
PluginNetworkHandler::client_connected(unsigned int clid)
{
}


void
PluginNetworkHandler::client_disconnected(unsigned int clid)
{
  subscribers_.remove_locked(clid);
}

void
PluginNetworkHandler::plugin_loaded(const char *plugin_name)
{
  send_loaded(plugin_name);
}

void
PluginNetworkHandler::plugin_unloaded(const char *plugin_name)
{
  send_unloaded(plugin_name);
}

} // end namespace fawkes
