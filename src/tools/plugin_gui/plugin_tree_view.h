
/***************************************************************************
 *  plugin_tree_view.h - Displays a list of Fawkes plugins and allows to
 *                       start/stop them
 *
 *  Created: Fri Sep 26 21:06:37 2008
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

#ifndef __TOOLS_CONFIG_GUI_PLUGIN_TREE_VIEW_H_
#define __TOOLS_CONFIG_GUI_PLUGIN_TREE_VIEW_H_

#include <netcomm/fawkes/client_handler.h>
#include <core/utils/lock_queue.h>

#include <gtkmm.h>
#include <libglademm/xml.h>

namespace fawkes {
  class FawkesNetworkClient;
}

class PluginTreeView 
: public Gtk::TreeView,
  public fawkes::FawkesNetworkClientHandler
{
 public:
  PluginTreeView(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml> ref_xml);
  virtual ~PluginTreeView();

  void set_network_client(fawkes::FawkesNetworkClient* client);

  // client handler
  void deregistered(unsigned int id) throw();
  void connection_died(unsigned int id) throw();
  void connection_established(unsigned int id) throw();
  void inbound_received(fawkes::FawkesNetworkMessage* m,
			unsigned int id) throw();

 protected:

  class PluginRecord : public Gtk::TreeModelColumnRecord
  {
  public:
    PluginRecord() 
      { 
	add(index);
	add(name);
	add(loaded);
      }
    
    Gtk::TreeModelColumn<int> index;           /**< an index */
    Gtk::TreeModelColumn<Glib::ustring> name;  /**< the name of the plugin */
    Gtk::TreeModelColumn<bool> loaded;         /**< the loaded status of the plugin */
  };

  enum PluginDataEventType
  {
    EVENT_TYPE_LOAD,       /**< the event contains information about the loaded status of a plugin */
    EVENT_TYPE_LIST,       /**< the event is part of a list */
    EVENT_TYPE_LIST_START  /**< the event marks the beginning of a list */
  };

  struct PluginData
  {
    PluginDataEventType event_type;  /**< the type of the event */
    std::string name;                /**< the name of the plugin */
    bool loaded;                     /**< the loaded status of the plugin */
  };

  fawkes::LockQueue<PluginData> m_incoming_plugin_data;

  Glib::Dispatcher m_signal_incoming_plugin_data;
  Glib::Dispatcher m_signal_connection_terminated;

  void on_incoming_plugin_data();
  void on_status_toggled(const Glib::ustring& path);
  void on_connection_terminated();

  Glib::RefPtr<Gtk::ListStore> m_plugin_list;
  PluginRecord m_plugin_record;

  fawkes::FawkesNetworkClient* m_client;
};

#endif /*  __TOOLS_CONFIG_GUI_PLUGIN_TREE_VIEW_H_ */
