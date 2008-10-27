
/***************************************************************************
 *  plugin_tree_view.cpp - Displays a list of Fawkes plugins and allows to
 *                         start/stop them
 *
 *  Created: Fri Sep 26 21:13:48 2008
 *  Copyright  2008  Daniel Beck
 *
 *  $Id: plugin_gui.h 899 2008-04-10 11:36:58Z tim $
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

#include <tools/plugin_gui/plugin_tree_view.h>
#include <netcomm/fawkes/client.h>
#include <plugin/net/messages.h>
#include <plugin/net/list_message.h>

#include <cstring>
#include <string>

using namespace std;
using namespace fawkes;

/** @class PluginTreeView tools/plugin_gui/plugin_tree_view.h
 * A TreeView class to list available plugins und trigger their
 * loading/unloading.
 *
 * @author Daniel Beck
 */

/** @class PluginTreeView::PluginRecord tools/plugin_gui/plugin_tree_view.h
 * Column record class for the plugin tree view.
 *
 * @author Daniel Beck
 */

/** @struct PluginTreeView::PluginData
 * This data structure holds incoming plugin data.
 */

/** @enum PluginTreeView::PluginDataEventType
 * Different type of incoming plugin data events.
 */

/** @var PluginTreeView::m_incoming_plugin_data
 * Queue for incoming plugin data.
 */

/** @var PluginTreeView::m_signal_incoming_plugin_data
 * This signal is emitted whenever new plugin data is available.
 */

/** @var PluginTreeView::m_signal_connection_terminated
 * This signal is emitted whenever the network connection is
 * terminated.
 */

/** @var PluginTreeView::m_plugin_list
 * Storage object for the plugin data.
 */

/** @var PluginTreeView::m_plugin_record
 * Column record object.
 */

/** @var PluginTreeView::m_client
 * The Fawkes network client.
 */

/** Constructor.
 * @param cobject pointer to base object type
 * @param ref_xml Glade XML file
 */
PluginTreeView::PluginTreeView( BaseObjectType* cobject,
				const Glib::RefPtr<Gnome::Glade::Xml> ref_xml )
  : Gtk::TreeView(cobject)
{
  m_plugin_list = Gtk::ListStore::create(m_plugin_record);
  set_model(m_plugin_list);
  append_column("#", m_plugin_record.index);
  append_column_editable("Status", m_plugin_record.loaded);
  append_column("Plugin", m_plugin_record.name);

  Gtk::CellRendererToggle* renderer;
  renderer = dynamic_cast<Gtk::CellRendererToggle*>( get_column_cell_renderer(1) );
  renderer->signal_toggled().connect( sigc::mem_fun(*this, &PluginTreeView::on_status_toggled));

  m_signal_incoming_plugin_data.connect( sigc::mem_fun(*this, &PluginTreeView::on_incoming_plugin_data) );
  m_signal_connection_terminated.connect( sigc::mem_fun(*this, &PluginTreeView::on_connection_terminated) );

  m_client = NULL;
}

/** Destructor. */
PluginTreeView::~PluginTreeView()
{
  if ( m_client->connected() )
  {
    // unsubscribe
    FawkesNetworkMessage* msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
							 MSG_PLUGIN_UNSUBSCRIBE_WATCH);
    m_client->enqueue(msg);
    msg->unref();			

    m_client->deregister_handler(FAWKES_CID_PLUGINMANAGER);
  }
}

/** Set the network client.
 * @param client a Fawkes network client
 */
void
PluginTreeView::set_network_client(FawkesNetworkClient* client)
{
  m_client = client;

  m_client->register_handler(this, FAWKES_CID_PLUGINMANAGER);

  try
    {
      if ( !m_client->connected() )
	{ m_client->connect(); }

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
    }
}

void
PluginTreeView:: deregistered(unsigned int id) throw()
{
}

void
PluginTreeView::connection_died(unsigned int id) throw()
{
  m_signal_connection_terminated();
}

void
PluginTreeView::connection_established(unsigned int id) throw()
{
}

void
PluginTreeView::inbound_received(fawkes::FawkesNetworkMessage* msg,
				 unsigned int id) throw()
{
  bool update = false;

  if (msg->cid() != FAWKES_CID_PLUGINMANAGER)
    { return; }

  // loading
  if ( msg->msgid() == MSG_PLUGIN_LOADED )
    {
      if ( msg->payload_size() != sizeof(plugin_loaded_msg_t) ) 
	{ printf("Invalid message size (load succeeded)\n"); } 
      else 
	{
	  plugin_loaded_msg_t* m = (plugin_loaded_msg_t*) msg->payload();

	  PluginData d;
	  d.event_type = EVENT_TYPE_LOAD;
	  d.name = string(m->name);
	  d.loaded = true;

	  m_incoming_plugin_data.push_locked(d);

	  update = true;
	}
    }

  // loading failed
  else if ( msg->msgid() == MSG_PLUGIN_LOAD_FAILED ) 
    {
      if ( msg->payload_size() != sizeof(plugin_load_failed_msg_t) ) 
	{ printf("Invalid message size (load failed)\n"); } 
      else 
	{
	  plugin_load_failed_msg_t* m = (plugin_load_failed_msg_t*) msg->payload();

	  PluginData d;
	  d.event_type = EVENT_TYPE_LOAD;
	  d.name = string(m->name);
	  d.loaded = false;
	  
	  m_incoming_plugin_data.push_locked(d);

	  update = true;
	}
    }

  // unloading
  else if ( msg->msgid() == MSG_PLUGIN_UNLOADED ) 
    {
      if ( msg->payload_size() != sizeof(plugin_unloaded_msg_t) ) 
	{ printf("Invalid message size (unload succeeded)\n"); } 
      else 
	{
	  plugin_unloaded_msg_t* m = (plugin_unloaded_msg_t*) msg->payload();

	  PluginData d;
	  d.event_type = EVENT_TYPE_LOAD;
	  d.name = string(m->name);
	  d.loaded = false;

	  m_incoming_plugin_data.push_locked(d);

	  update = true;
	}
    } 
  
  // unloading failed
  else if ( msg->msgid() == MSG_PLUGIN_UNLOAD_FAILED) 
    {
      if ( msg->payload_size() != sizeof(plugin_unload_failed_msg_t) ) 
	{ printf("Invalid message size (unload failed)\n"); } 
      else 
	{
	  plugin_unload_failed_msg_t* m = (plugin_unload_failed_msg_t*) msg->payload();

	  PluginData d;
	  d.event_type = EVENT_TYPE_LOAD;
	  d.name = string(m->name);
	  d.loaded = true;

	  m_incoming_plugin_data.push_locked(d);

	  update = true;
	}
    }

  // list available plugins
  else if (msg->msgid() == MSG_PLUGIN_AVAIL_LIST ) 
    {
      bool first = true;;
      PluginListMessage* plm = msg->msgc<PluginListMessage>();
      while ( plm->has_next() ) 
	{
	  char* name = plm->next();

	  PluginData d;
	  if (first)
	    { 
	      d.event_type = EVENT_TYPE_LIST_START;
	      first = false;
	    }
	  else
	    { d.event_type = EVENT_TYPE_LIST; }
	  d.name = string(name);
	  d.loaded = false;

	  m_incoming_plugin_data.push_locked(d);

	  free(name);
	}
      delete plm;

      update = true;
    } 

  else if ( msg->msgid() == MSG_PLUGIN_AVAIL_LIST_FAILED) 
    { printf("Obtaining list of available plugins failed\n"); }


  // list loaded plugins
  else if (msg->msgid() == MSG_PLUGIN_LOADED_LIST ) 
    {
      bool first = true;
      PluginListMessage* plm = msg->msgc<PluginListMessage>();
      while ( plm->has_next() ) 
	{
	  char* name = plm->next();
	  
	  PluginData d;
	  if (first)
	    {
	      d.event_type = EVENT_TYPE_LIST_START;
	      first = false;
	    }
	  else
	    { d.event_type = EVENT_TYPE_LIST; }
	  d.name = string(name);
	  d.loaded = true;

	  m_incoming_plugin_data.push_locked(d);
	  
	  free(name);
	}
      delete plm;
      
      update = true;
    } 
  
  else if ( msg->msgid() == MSG_PLUGIN_LOADED_LIST_FAILED) 
    { printf("Obtaining list of loaded plugins failed\n"); }

  // unknown message received
  else
    {
      printf("received message with msg-id %d\n", msg->msgid());
    }

  if (update)
    { m_signal_incoming_plugin_data(); }
}

/** Signal handler that is called when the loaded checkbox is
 * toggled.
 * @param path the path of the selected row
 */
void
PluginTreeView::on_status_toggled(const Glib::ustring& path)
{
  Gtk::TreeModel::Row row = *m_plugin_list->get_iter(path);
  Glib::ustring plugin_name = row[m_plugin_record.name];
  bool loaded = row[m_plugin_record.loaded];

  if ( !m_client->connected() )
    { return; }

  if (loaded)
    {
      plugin_load_msg_t* m = (plugin_load_msg_t*) calloc(1, sizeof(plugin_load_msg_t));
      strncpy(m->name, plugin_name.c_str(), PLUGIN_MSG_NAME_LENGTH);
      
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
							   MSG_PLUGIN_LOAD,
							   m, sizeof(plugin_load_msg_t));
      m_client->enqueue(msg);
      msg->unref();
    }
  else
    {
      plugin_unload_msg_t* m = (plugin_unload_msg_t *)calloc(1, sizeof(plugin_unload_msg_t));
      strncpy(m->name, plugin_name.c_str(), PLUGIN_MSG_NAME_LENGTH);
      
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
							   MSG_PLUGIN_UNLOAD,
							   m, sizeof(plugin_unload_msg_t));
      m_client->enqueue(msg);
      msg->unref();
    }
}

/** Signal handler that manages the incoming data. */
void
PluginTreeView::on_incoming_plugin_data()
{
  m_incoming_plugin_data.lock();

  while ( !m_incoming_plugin_data.empty() )
    {
      PluginData& d = m_incoming_plugin_data.front();

      switch (d.event_type)
	{
	case EVENT_TYPE_LOAD:
	  {
	    Gtk::TreeIter iter;
	    for ( iter  = m_plugin_list->children().begin();
		  iter != m_plugin_list->children().end();
		  ++iter )
	      {
		Glib::ustring n = (*iter)[m_plugin_record.name];
		if ( n == d.name )
		{
		  (*iter)[m_plugin_record.loaded] = d.loaded;
		  break;
		}
	      }

	    break;
	  }

	case EVENT_TYPE_LIST_START:
	  m_plugin_list->clear();

	case EVENT_TYPE_LIST:
	  {
	    Gtk::TreeModel::Row row = *m_plugin_list->append();
	    unsigned int index = m_plugin_list->children().size();
	    row[m_plugin_record.index]  = index;
	    row[m_plugin_record.name]   = d.name;
	    row[m_plugin_record.loaded] = d.loaded;
	    	    
	    break;
	  }

	default:
	  break;	
	}

      m_incoming_plugin_data.pop();
    }

  m_incoming_plugin_data.unlock();
}

/** Signal handler that is called whenever the connection is terminated. */
void
PluginTreeView::on_connection_terminated()
{
  m_plugin_list->clear();
}
