
/***************************************************************************
 *  plugin_gui.cpp -  Plugin Tool Gui
 *
 *  Created: Thu Nov 09 20:16:23 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <tools/plugin_gui/plugin_gui.h>
#include <tools/plugin_gui/backend_thread.h>

#include <iostream>

using namespace std;

/** @class PluginGui tools/plugin_gui/plugin_gui.h
 * Graphical plugin management tool.
 * 
 * @author Daniel Beck
 */

/** Constructor.
 * @param ref_xml RefPtr to the Glade xml file
 */
PluginGui::PluginGui(Glib::RefPtr<Gnome::Glade::Xml> ref_xml)
{
  m_wnd_main = 0;
  m_cbe_hosts = 0;
  m_btn_connect = 0;
  m_trv_plugins = 0;
  m_stb_status = 0;
  m_backend = 0;

  ref_xml->get_widget("wndMain", m_wnd_main);
  if (!m_wnd_main)
    {
      throw runtime_error("Couldn't find wndMain");
    }

  ref_xml->get_widget("cbeHosts", m_cbe_hosts);
  if (!m_cbe_hosts)
    {
      throw runtime_error("Couldn't find cbeHosts");
    }

  m_host_list = Gtk::ListStore::create(m_host_record);
  m_cbe_hosts->set_model(m_host_list);
  m_cbe_hosts->set_text_column(m_host_record.m_host);

  ref_xml->get_widget("btnConnect", m_btn_connect);
  if (!m_btn_connect)
    {
      throw runtime_error("Couldn't find btnConnect");
    }
  m_btn_connect->set_use_stock(true);

  ref_xml->get_widget("trvPlugins", m_trv_plugins);
    if ( !m_trv_plugins )
      {
	throw runtime_error("Couldn't find trvPlugins");
      }

  m_plugin_list = Gtk::ListStore::create(m_plugin_record);
  m_trv_plugins->set_model(m_plugin_list);
  m_trv_plugins->append_column("#", m_plugin_record.m_index);
  m_trv_plugins->append_column_editable("Status", m_plugin_record.m_status);
  m_trv_plugins->append_column("Plugin", m_plugin_record.m_name);

  ref_xml->get_widget("stbStatus", m_stb_status);
  if (!m_stb_status)
    {
      throw runtime_error("Couldn't find stbStatus");
    }

  m_btn_connect->signal_clicked().connect( sigc::mem_fun( *this, &PluginGui::clicked_connect ) );
  m_cbe_hosts->get_entry()->set_activates_default(true);
  m_cbe_hosts->signal_changed().connect( sigc::mem_fun( *this, &PluginGui::changed_host ) );
  
  Gtk::CellRendererToggle* cr_loaded;
  cr_loaded = dynamic_cast<Gtk::CellRendererToggle*>(m_trv_plugins->get_column_cell_renderer(1));
  cr_loaded->signal_toggled().connect( sigc::mem_fun(*this, &PluginGui::toggled_status));

  m_signal_update_status.connect( sigc::mem_fun( *this, &PluginGui::update_status) );
  m_signal_update_list.connect( sigc::mem_fun( *this, &PluginGui::update_list) );
  m_signal_update_hosts.connect( sigc::mem_fun( *this, &PluginGui::update_hosts) );
  m_signal_update_connection.connect( sigc::mem_fun( *this, &PluginGui::update_connection) );

  m_stb_status->push("Started");
}

/** Destructor. */
PluginGui::~PluginGui()
{
  m_stb_status->push("Exiting");
  if (m_backend)
    {
      m_backend->cancel();
      m_backend->join();
      delete m_backend;
    }
}

/** Connect to backend thread.
 * @param backend point to the backend thread
 */
void
PluginGui::register_backend(PluginGuiBackendThread* backend)
{
  m_backend = backend;
  m_backend->start();
}

/** Return main window of the application.
 * @return reference to the main window
 */
Gtk::Window&
PluginGui::get_window() const
{
  return *m_wnd_main;
}

/** Signal that the status has changed. */
void
PluginGui::signal_update_status()
{
  m_signal_update_status();
}

/** Update the plugins' status.
 * This method is called by the backend when the status of some plugin has changed.
 */
void
PluginGui::update_status()
{
  std::map<std::string,bool>& plugin_status = m_backend->plugin_status();
  Gtk::TreeIter row_iter;
  Glib::ustring plugin_name;
  bool status;
  for (row_iter = m_plugin_list->children().begin();
       row_iter != m_plugin_list->children().end();
       row_iter++)
    {
      plugin_name = (*row_iter)[m_plugin_record.m_name];
      status = plugin_status[plugin_name.c_str()];
      (*row_iter)[m_plugin_record.m_status] = status;
    }

  m_trv_plugins->queue_draw();  
}

/** Signal that the list needs an update. */
void
PluginGui::signal_update_list()
{
  m_signal_update_list();
}

/** Update the list of available plugins.
 * This method is called by the backend when the available plugins have changed. 
 */
void
PluginGui::update_list()
{
  unsigned int index = 1;
  std::map<std::string,bool>::iterator pit;
  std::map<std::string,bool>& plugin_status = m_backend->plugin_status();
  for (pit = plugin_status.begin(); pit != plugin_status.end(); pit++)
    {
      Gtk::TreeModel::Row row = *m_plugin_list->append();
      row[m_plugin_record.m_index] = index;
      row[m_plugin_record.m_name] = pit->first;
      row[m_plugin_record.m_status] = pit->second;
      ++index;
    }
  
  m_trv_plugins->queue_draw();
}

/** Signal that the hosts have changed. */
void
PluginGui::signal_update_hosts()
{
  m_signal_update_hosts();
}

/** Update the list of discovered hosts.
 * This method is called by the backend when new host are discovered or hosts
 * are removed.
 */
void
PluginGui::update_hosts()
{
  unsigned int index = 1;
  std::vector<std::string> hosts = m_backend->hosts();
  std::vector<std::string>::iterator hit;

  m_host_list->clear();

  for (hit = hosts.begin(); hit != hosts.end(); hit++)
    {
      Gtk::TreeModel::Row row = *m_host_list->append();
      row[m_host_record.m_index] = index;
      row[m_host_record.m_host] = *hit;
      ++index;
    }
}

/** Signal that the connection status has changed. */
void
PluginGui::signal_update_connection()
{
  m_signal_update_connection();
}

/** Update the connection status.
 * This method is called by the backend when the connection status changes.
 */
void
PluginGui::update_connection()
{
  if ( m_backend->is_connected() )
    {
      m_stb_status->push("Connected to " + m_cbe_hosts->get_entry()->get_text());
      m_btn_connect->set_label(Gtk::Stock::DISCONNECT.id);
    }
  else if ( !m_backend->is_connected() )
    {
      m_plugin_list->clear();
      m_stb_status->push("Connection died");
      m_btn_connect->set_label(Gtk::Stock::CONNECT.id);
      //      m_cbe_hosts->get_entry()->set_text("");
    }
}

/** Signal handler.
 * Connects to the specified host or disconnects in case it were connected.
 */
void
PluginGui::clicked_connect()
{
  if ( m_backend->is_connected() )
    {
      m_plugin_list->clear();
      m_stb_status->push("Disconnecting ...");
      m_backend->disconnect();
      m_stb_status->push("Disconnected");
      //      m_cbe_hosts->get_entry()->set_text("");
      m_btn_connect->set_label(Gtk::Stock::CONNECT.id);
    }
  else
    {
      Glib::ustring host = m_cbe_hosts->get_entry()->get_text();
      if ( !host.empty() )
	{
	  m_stb_status->push("Connecting to " + m_cbe_hosts->get_entry()->get_text());
	  m_backend->connect(m_cbe_hosts->get_entry()->get_text().c_str(), 1910);
	}
    }
}

/** Signal handler.
 * Connects to the specified host (from the combo box).
 */
void
PluginGui::changed_host()
{
  if ( -1 == m_cbe_hosts->get_active_row_number() ) 
    {
      return;
    }

  if ( m_backend->is_connected() )
    {
      m_plugin_list->clear();
      m_backend->disconnect();
      m_btn_connect->set_label(Gtk::Stock::CONNECT.id);      
    }
  Glib::ustring host = m_cbe_hosts->get_entry()->get_text();
  if ( !host.empty() )
    {
      m_stb_status->push("Connecting to " + m_cbe_hosts->get_entry()->get_text());
      m_backend->connect(m_cbe_hosts->get_entry()->get_text().c_str(), 1910);
    }
}

/** Signal handler.
 * Triggers (un-)loading of plugins.
 * @param path path to the corresponding row
 */
void
PluginGui::toggled_status(const Glib::ustring path)
{
  Gtk::TreeModel::Row row = *m_plugin_list->get_iter(path);
  Glib::ustring plugin_name = row[m_plugin_record.m_name];
  bool loaded = row[m_plugin_record.m_status];
  if (loaded)
    {
      m_backend->request_load(plugin_name.c_str());
    }
  else
    {
      m_backend->request_unload(plugin_name.c_str());
    }
}
