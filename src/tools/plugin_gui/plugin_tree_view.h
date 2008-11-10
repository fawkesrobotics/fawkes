
/***************************************************************************
 *  plugin_tree_view.h - Displays a list of Fawkes plugins and allows to
 *                       start/stop them
 *
 *  Created: Fri Sep 26 21:06:37 2008
 *  Copyright  2008  Daniel Beck
 *             2008  Tim Niemueller [www.niemueller.de]
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
#include <gui_utils/connection_dispatcher.h>

#include <gtkmm.h>
#include <libglademm/xml.h>

namespace fawkes {
  class FawkesNetworkClient;
  class FawkesNetworkMessage;
}

class PluginTreeView 
: public Gtk::TreeView
{
 public:
  PluginTreeView(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml> ref_xml);
  virtual ~PluginTreeView();

  void set_network_client(fawkes::FawkesNetworkClient* client);

 private:
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

  void on_status_toggled(const Glib::ustring& path);
  void on_connected();
  void on_disconnected();
  void on_message_received(fawkes::FawkesNetworkMessage *msg);

 private:
  Glib::RefPtr<Gtk::ListStore> m_plugin_list;
  PluginRecord m_plugin_record;

  fawkes::ConnectionDispatcher m_dispatcher;
};

#endif /*  __TOOLS_CONFIG_GUI_PLUGIN_TREE_VIEW_H_ */
