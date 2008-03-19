
/***************************************************************************
 *  netlog_monitor.h - Fawkes Network Log Monitor
 *
 *  Created: Sun Dec 09 20:15:09 2007
 *  Copyright  2007  Daniel Beck
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

#ifndef __TOOLS_NETLOG_MONITOR_NETLOG_MONITOR_H_
#define __TOOLS_NETLOG_MONITOR_NETLOG_MONITOR_H_

#include <utils/logging/logger.h>
#include <core/utils/lock_queue.h>

#include <gtkmm.h>
#include <libglademm/xml.h>

class NetLogMonitorBackendThread;

class NetLogMonitor : public Gtk::Window
{
 public:
  NetLogMonitor(Glib::RefPtr<Gnome::Glade::Xml> ref_xml);
  virtual ~NetLogMonitor();

  Gtk::Window& get_window() const;
  
  void signal_conn_status_chg(unsigned int client_id, bool connected);
  void signal_new_host(unsigned int client_id, std::string host_name);
  void signal_new_message(unsigned int client_id, const char* loglevel,
			  const char* time, const char* component,
			  const char* message);

  void register_backend(NetLogMonitorBackendThread* backend);

 private:
  class LogRecord : public Gtk::TreeModelColumnRecord
    {
    public:
      LogRecord()
	{
	  add(m_loglevel);
	  add(m_time);
	  add(m_component);
	  add(m_message);
	}

      Gtk::TreeModelColumn<Glib::ustring> m_loglevel;
      Gtk::TreeModelColumn<Glib::ustring> m_time;
      Gtk::TreeModelColumn<Glib::ustring> m_component;
      Gtk::TreeModelColumn<Glib::ustring> m_message;
    };

  Gtk::Widget* get_widget(Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
			   const char* widget_name) const;

  // signal handler
  void connect();
  void exit();
  void clear();
  
  void close_page();

  void conn_status_chg();
  void new_host();
  void new_message();

  // widgets
  Gtk::Window* m_wnd_main;
  Gtk::ToolButton* m_tbtn_clear;
  Gtk::ToolButton* m_tbtn_connect;
  Gtk::ToolButton* m_tbtn_exit;
  Gtk::Entry* m_ent_host;
  Gtk::Notebook* m_ntb_hosts;
  Gtk::Statusbar* m_stb_status;

  Glib::Dispatcher m_signal_new_message;
  Glib::Dispatcher m_signal_conn_status_chg;
  Glib::Dispatcher m_signal_new_host;

  // client id, host name
  LockQueue< std::pair<unsigned int, std::string> > m_new_hosts;
  // client id, new conn status
  LockQueue< std::pair<unsigned int, bool> > m_conn_status_chg;

  // host name, page number
  std::map<std::string, int> m_pages;
  // page number, tab label
  std::map<int, Gtk::Label*> m_tab_labels;
  // client id, host name
  std::map<unsigned int, std::string> m_host_names;
  // page num, list store
  std::map<int, Glib::RefPtr<Gtk::ListStore> > m_message_lists;


  LogRecord m_log_record;

  NetLogMonitorBackendThread* m_backend;
};

#endif /* __TOOLS_PLUGIN_PLUGIN_GUI_PLUGIN_GUI_H_ */
