
/***************************************************************************
 *  netlog_monitor.h - Fawkes Network Log Monitor
 *
 *  Created: Sun Dec 09 20:26:31 2007
 *  Copyright  2007  Daniel Beck
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

#include <tools/netlog_monitor/netlog_monitor.h>
#include <tools/netlog_monitor/backend_thread.h>

#include <utils/logging/logger.h>

using namespace std;
using namespace fawkes;


/** @class NetLogMonitor tools/netlog_monitor/netlog_monitor.h
 * Graphical frontend for the network logger.
 *
 * @author Daniel Beck
 */


/** Constructor.
 * @param ref_xml reference pointer to the Glade XML file
 */
NetLogMonitor::NetLogMonitor(Glib::RefPtr<Gnome::Glade::Xml> ref_xml)
{
  m_backend = 0;

  m_pages.clear();
  m_tab_labels.clear();
  m_new_hosts.clear();
  m_conn_status_chg.clear();

  m_wnd_main = dynamic_cast<Gtk::Window*>( get_widget(ref_xml, "wndMain") );
  m_ntb_hosts = dynamic_cast<Gtk::Notebook*>( get_widget(ref_xml, "ntbHosts") );
  m_tbtn_clear = dynamic_cast<Gtk::ToolButton*>( get_widget(ref_xml, "tbtnClear") );
  m_tbtn_connect = dynamic_cast<Gtk::ToolButton*>( get_widget(ref_xml, "tbtnConnect") );
  m_tbtn_exit = dynamic_cast<Gtk::ToolButton*>( get_widget(ref_xml, "tbtnExit") );
  m_ent_host = dynamic_cast<Gtk::Entry*>( get_widget(ref_xml, "entHost") );
  m_stb_status = dynamic_cast<Gtk::Statusbar*>( get_widget(ref_xml, "stbStatus") );

  m_ent_host->set_activates_default(true);

  m_tbtn_clear->signal_clicked().connect( sigc::mem_fun( *this, &NetLogMonitor::clear) );
  m_tbtn_connect->signal_clicked().connect( sigc::mem_fun( *this, &NetLogMonitor::connect ) );
  m_tbtn_exit->signal_clicked().connect( sigc::mem_fun( *this, &NetLogMonitor::exit) );

  m_ntb_hosts->remove_page();

  m_signal_new_message.connect( sigc::mem_fun( *this, &NetLogMonitor::new_message ) );
  m_signal_conn_status_chg.connect( sigc::mem_fun( *this, &NetLogMonitor::conn_status_chg) );
  m_signal_new_host.connect( sigc::mem_fun( *this, &NetLogMonitor::new_host) );
}

/** Destructor. */
NetLogMonitor::~NetLogMonitor()
{
}

Gtk::Widget*
NetLogMonitor::get_widget(Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
			  const char* widget_name) const
{
  Gtk::Widget* widget;
  ref_xml->get_widget(widget_name, widget);
  if ( !widget )
    { 
      char* err_str;
      asprintf(&err_str, "Couldn't find widget %s", widget_name);
      throw runtime_error(err_str);
      free(err_str);
    }

  return widget;
}

/** Returns reference to the main window.
 * @return reference to the main window
 */
Gtk::Window&
NetLogMonitor::get_window() const
{
  return *m_wnd_main;
}

void
NetLogMonitor::exit()
{
  m_stb_status->push("Exiting");

  if (m_backend)
    {
      m_backend->cancel();
      m_backend->join();
      delete m_backend;
    }

  m_wnd_main->hide();
}

/** Signal that the connection status has changed.
 * @param client_id the id of the client which changed its connection status
 * @param connected true if status changed to connected, false if status
 *        changed to disconnected
 */
void
NetLogMonitor::signal_conn_status_chg(unsigned int client_id, bool connected)
{
  std::pair<unsigned int, bool> elem(client_id, connected);
  m_conn_status_chg.push_locked(elem);

  m_signal_conn_status_chg();
}

/** Update the connection status. */
void
NetLogMonitor::conn_status_chg()
{
  m_conn_status_chg.lock();
  if ( m_conn_status_chg.empty() ) 
    {
      m_conn_status_chg.unlock();
      return;
    }

  std::pair<unsigned int, bool> elem = m_conn_status_chg.front();
  m_conn_status_chg.pop();
  m_conn_status_chg.unlock();

  unsigned int id = elem.first;
  bool connected = elem.second;
  
  int page_num = m_pages[ m_host_names[id] ];
  Gtk::Label* tab_label = m_tab_labels[page_num];

  Glib::ustring new_label;
  if ( !connected )
    {
      new_label  = "<i>";
      new_label += Glib::ustring( m_host_names[id].c_str() );
      new_label += "</i>";
    }
  else
    {
      new_label  = "<b>";
      new_label += Glib::ustring( m_host_names[id].c_str() );
      new_label += "</b>";
    }

  tab_label->set_markup(new_label);
}

/** Signal that the list of hosts running Fawkes has changed.
 * @param client_id the id of the new client
 * @param host_name the host name of the new host
 */
void
NetLogMonitor::signal_new_host(unsigned int client_id,
			       std::string host_name)
{
  std::pair< unsigned int, std::string> elem(client_id, host_name);
  m_new_hosts.push_locked(elem);
  m_signal_new_host();
}

/** Update the list of available hosts. */
void
NetLogMonitor::new_host()
{
  m_new_hosts.lock();
  if ( m_new_hosts.empty() )
    {
      m_new_hosts.unlock();
      return;
    }
  
  std::pair<unsigned int, std::string> elem = m_new_hosts.front();
  m_new_hosts.pop();
  m_new_hosts.unlock();

  unsigned int id = elem.first;
  std::string host_name = elem.second;

  if ( m_pages.find(host_name) != m_pages.end() )
    { 
      m_host_names[id] = host_name;
      signal_conn_status_chg(id, true);
      return; 
    }

  int page_num;
  Gtk::HBox* tab_label = new Gtk::HBox;
  Gtk::Button* btn_close = new Gtk::Button("x");
  btn_close->set_relief(Gtk::RELIEF_NONE);
  Gtk::Label* lbl_host = new Gtk::Label(Glib::ustring(host_name.c_str()));
  tab_label->add(*btn_close);
  tab_label->add(*lbl_host);
  tab_label->show_all_children();
  
  Glib::RefPtr<Gtk::ListStore> list = Gtk::ListStore::create(m_log_record);
  Gtk::TreeView* child = new Gtk::TreeView();
  child->set_model(list);
  child->append_column("Level", m_log_record.m_loglevel);
  child->append_column("Time", m_log_record.m_time);
  child->append_column("Component", m_log_record.m_component);
  child->append_column("Message", m_log_record.m_message);

  page_num = m_ntb_hosts->append_page(*child, *tab_label);

  m_message_lists[page_num] = list;
  m_tab_labels[page_num] = lbl_host;
  m_pages[host_name] = page_num;
  m_host_names[id] = host_name;

  m_ntb_hosts->show_all_children();
} 


/** Signal that new log messages are available. 
 * @param client_id the id of the client that received the log message
 * @param loglevel string containing description of the log level
 * @param time time string
 * @param component the component
 * @param message the message
 */
void
NetLogMonitor::signal_new_message(unsigned int client_id, const char* loglevel,
				  const char* time, const char* component,
				  const char* message)
{
  int page_num = m_pages[ m_host_names[client_id] ];

  Gtk::TreeModel::Row row = *(m_message_lists[page_num])->append();
  row[m_log_record.m_loglevel] = Glib::ustring(loglevel);
  row[m_log_record.m_time] = Glib::ustring(time);
  row[m_log_record.m_component] = Glib::ustring(component);
  row[m_log_record.m_message] = Glib::ustring(message);
  
  if ( page_num == m_ntb_hosts->get_current_page() )
    { m_signal_new_message(); }
}

/** Get the new log messages from the backend thread and display them. */
void
NetLogMonitor::new_message()
{
  m_ntb_hosts->queue_draw();
}

/** Clears the log messages. */
void
NetLogMonitor::clear()
{
}

/** Signal handler for the connect button. */
void
NetLogMonitor::connect()
{
  if ( !m_backend )
    { return; }

  m_backend->connect( m_ent_host->get_text().c_str() );
}

/** Register the backend.
 * @param backend pointer to the backend
 */
void
NetLogMonitor::register_backend(NetLogMonitorBackendThread* backend)
{
  m_backend = backend;
  m_backend->start();
}
