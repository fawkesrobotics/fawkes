
/***************************************************************************
 *  backend_thread.cpp - Backend thread of the Plugin Tool Gui
 *
 *  Created: Wed Nov 08 09:54:04 2007
 *  Copyright  2007  Daniel Beck
 *             2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <tools/plugin_gui/backend_thread.h>
#include <tools/plugin_gui/plugin_gui.h>
#include <core/threading/wait_condition.h>
#include <netcomm/utils/exceptions.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/dns-sd/avahi_thread.h>
#include <mainapp/plugin_messages.h>
#include <mainapp/plugin_list_message.h>
#include <iostream>
#include <cstring>

using namespace std;
using namespace fawkes;

/** @class PluginGuiBackendThread backend_thread.h <tools/plugin_gui/backend_thread.h>
 * Backend-thread for the graphical plugin management tool.
 */

/** Constructor.
 * @param gui a point to the corresponding GUI instance
 *
 * @author Daniel Beck
 * @author Tim Niemueller (copied some code from the console plugin tool)
 */
PluginGuiBackendThread::PluginGuiBackendThread(PluginGui* gui)
  : Thread("PluginGuiBackendThread")
{
  m_client = NULL;
  m_avahi = new AvahiThread();
  m_avahi->watch_service("_fawkes._tcp", this);
  m_avahi->start();
  m_connected = false;
  m_connection_died = false;
  m_gui = gui;
  m_hosts.clear();
  m_longsleep = new WaitCondition();
}

/** Desctructor. */
PluginGuiBackendThread::~PluginGuiBackendThread()
{
  m_avahi->cancel();
  m_avahi->join();
  disconnect();
  delete m_client;
  delete m_longsleep;
}

/** Connect to the host running Fawkes.
 * @param host the hostname
 * @param port th port number
 * @return true if connected
 */
bool
PluginGuiBackendThread::connect(const char* host, unsigned short int port)
{
  if (m_connected) { disconnect(); }

  m_client = new FawkesNetworkClient(host, port);
  m_client->register_handler(this, FAWKES_CID_PLUGINMANAGER);
  try
    {
      m_client->connect();

      m_connected = true;

      // subscribe for load-/unload messages
      FawkesNetworkMessage* msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
							   MSG_PLUGIN_SUBSCRIBE_WATCH);
      m_client->enqueue(msg);
      msg->unref();
      
      // request list of available plugins
      msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
				     MSG_PLUGIN_LIST_AVAIL);
      m_client->enqueue(msg);
      msg->unref();
      
      // request list of loaded plugins
      msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
				     MSG_PLUGIN_LIST_LOADED);
      m_client->enqueue(msg);
      msg->unref();
    }
  catch (Exception& e)
    {
      e.print_trace();
      m_connected = false;
    }

  m_longsleep->wake_all();
  return m_connected;
}

/** Disconnect. 
 * @return true if disconnected
 */
bool
PluginGuiBackendThread::disconnect()
{
  if (m_connected)
  {
    // unsubscribe
    FawkesNetworkMessage* msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
							 MSG_PLUGIN_UNSUBSCRIBE_WATCH);
    m_client->enqueue(msg);
    msg->unref();			

    m_connected = false;

    m_client->disconnect();
    m_client->deregister_handler(FAWKES_CID_PLUGINMANAGER);
    delete m_client;
    m_client = 0;
  }
  
  m_plugin_status.clear();

  return true;
}

/** Returns connection status.
 * @return true if connected, false otherwise
 */
bool
PluginGuiBackendThread::is_connected()
{
  return m_connected;
}

/** The backend-thread's loop.
 * Just waiting for incoming messages...
 */
void
PluginGuiBackendThread::loop()
{
  if (m_connected)
    {
      m_client->wait(FAWKES_CID_PLUGINMANAGER);
    }
  else
    {
      m_longsleep->wait();
    }

  if (m_connection_died)
    {
      m_client->disconnect();
      m_client->deregister_handler(FAWKES_CID_PLUGINMANAGER);
      m_connection_died = false;
      delete m_client;
      m_client = 0;
    }
}

void
PluginGuiBackendThread::deregistered(unsigned int id) throw()
{
}

void
PluginGuiBackendThread::connection_established(unsigned int id) throw()
{
  m_connected = true;
  m_gui->signal_update_connection();
}

void
PluginGuiBackendThread::connection_died(unsigned int id) throw()
{
  if (m_connected)
    // unexpected loss of connection
    {
      m_connected = false;
      m_gui->signal_update_connection();
      m_plugin_status.clear();
      m_gui->signal_update_list();
      m_connection_died = true;
    }
}

void
PluginGuiBackendThread::inbound_received(FawkesNetworkMessage* msg,
					 unsigned int id) throw()
{
  bool update = false;
  
  if (msg->cid() != FAWKES_CID_PLUGINMANAGER)
    {
      return;
    }

  // loading
  if ( msg->msgid() == MSG_PLUGIN_LOADED )
    {
      if ( msg->payload_size() != sizeof(plugin_loaded_msg_t) ) 
	{
	  printf("Invalid message size (load succeeded)\n");
	} 
      else 
	{
	  plugin_loaded_msg_t* m = (plugin_loaded_msg_t*) msg->payload();
	  //printf("Loading of %s succeeded\n", m->name);
	  m_plugin_status[std::string(m->name)] = true;
	  update = true;
	}
    } 

  // loading failed
  else if ( msg->msgid() == MSG_PLUGIN_LOAD_FAILED ) 
    {
      if ( msg->payload_size() != sizeof(plugin_load_failed_msg_t) ) 
	{
	  printf("Invalid message size (load failed)\n");
	} 
      else 
	{
	  plugin_load_failed_msg_t* m = (plugin_load_failed_msg_t*) msg->payload();
	  printf("Loading of %s failed, see log for reason\n", m->name);
	  update = true;
	}
    }

  // unloading
  else if ( msg->msgid() == MSG_PLUGIN_UNLOADED ) 
    {
      if ( msg->payload_size() != sizeof(plugin_unloaded_msg_t) ) 
	{
	  printf("Invalid message size (unload succeeded)\n");
	} 
      else 
	{
	  plugin_unloaded_msg_t* m = (plugin_unloaded_msg_t*) msg->payload();
	  //printf("Unloading of %s succeeded\n", m->name);
	  m_plugin_status[std::string(m->name)] = false;
	  update = true;
	}
    } 
  
  // unloading failed
  else if ( msg->msgid() == MSG_PLUGIN_UNLOAD_FAILED) 
    {
      if ( msg->payload_size() != sizeof(plugin_unload_failed_msg_t) ) 
	{
	  printf("Invalid message size (unload failed)\n");
	} 
      else 
	{
	  plugin_unload_failed_msg_t* m = (plugin_unload_failed_msg_t*) msg->payload();
	  printf("Unloading of %s failed, see log for reason\n", m->name);
	  update = true;
	}
    }

  // list available plugins
  else if (msg->msgid() == MSG_PLUGIN_AVAIL_LIST ) 
    {
      PluginListMessage* plm = msg->msgc<PluginListMessage>();
      while ( plm->has_next() ) 
	{
	  char* p = plm->next();
	  std::map<std::string,bool>::iterator iter = m_plugin_status.find(std::string(p));
	  if (iter == m_plugin_status.end())
	    {
	      m_plugin_status[std::string(p)] = false;
	    }
	  free(p);
	}
      m_gui->signal_update_list();
      delete plm;
    } 
  else if ( msg->msgid() == MSG_PLUGIN_AVAIL_LIST_FAILED) 
    {
      printf("Obtaining list of available plugins failed\n");
    }


  // list loaded plugins
  else if (msg->msgid() == MSG_PLUGIN_LOADED_LIST ) {
    PluginListMessage* plm = msg->msgc<PluginListMessage>();
    while ( plm->has_next() ) {
      char* p = plm->next();
      m_plugin_status[std::string(p)] = true;
      free(p);
    }
    update = true;
    delete plm;
  } 
  else if ( msg->msgid() == MSG_PLUGIN_LOADED_LIST_FAILED) 
    {
      printf("Obtaining list of loaded plugins failed\n");
    }

  // unknown message received
  else
    {
      printf("received message with msg-id %d\n", msg->msgid());
    }

  if (update)
    {
      m_gui->signal_update_status();
    }
  
}

void
PluginGuiBackendThread::all_for_now()
{
}

void
PluginGuiBackendThread::cache_exhausted()
{
}

void
PluginGuiBackendThread::browse_failed( const char* name,
				       const char* type,
				       const char* domain )
{
}

void
PluginGuiBackendThread::service_added( const char* name,
				       const char* type,
				       const char* domain,
				       const char* host_name,
				       const struct sockaddr* addr,
				       const socklen_t addr_size,
				       uint16_t port,
				       std::list<std::string>& txt,
				       int flags )
{
  m_hosts[name] = host_name;
  m_gui->signal_update_hosts();
}

void
PluginGuiBackendThread::service_removed( const char* name,
					 const char* type,
					 const char* domain )
{
  m_hosts.erase(name);
  m_gui->signal_update_hosts();
}

/** Get status of plugins.
 * @return a map associating plugin names with their loaded status
 */
std::map<std::string,bool>&
PluginGuiBackendThread::plugin_status()
{
  return m_plugin_status;
}

/** Return discovered hosts running Fawkes.
 * @return vector with hostnames
 */
std::vector<std::string>
PluginGuiBackendThread::hosts()
{
  std::vector<std::string> hosts;
  std::map<std::string,std::string>::iterator hit;
  for (hit = m_hosts.begin(); hit != m_hosts.end(); hit++)
    {
      hosts.push_back(hit->second);
    }
  return hosts;
}

/** Request loading of specified plugin.
 * @param plugin_name the name of the plugin
 */
void
PluginGuiBackendThread::request_load(const char* plugin_name)
{
  if (m_connected)
    {
      plugin_load_msg_t* m = (plugin_load_msg_t*) calloc(1, sizeof(plugin_load_msg_t));
      strncpy(m->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
      
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
							   MSG_PLUGIN_LOAD,
							   m, sizeof(plugin_load_msg_t));
      m_client->enqueue(msg);
      msg->unref();
    }
}

/** Request unloading of specified plugin.
 * @param plugin_name the name of the plugin
 */
void
PluginGuiBackendThread::request_unload(const char* plugin_name)
{
  if (m_connected)
    {
      plugin_unload_msg_t* m = (plugin_unload_msg_t *)calloc(1, sizeof(plugin_unload_msg_t));
      strncpy(m->name, plugin_name, PLUGIN_MSG_NAME_LENGTH);
      
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
							   MSG_PLUGIN_UNLOAD,
							   m, sizeof(plugin_unload_msg_t));
      m_client->enqueue(msg);
      msg->unref();
    }
}
