
/***************************************************************************
 *  plugin_tree_view.h - Displays a list of Fawkes plugins and allows to
 *                       start/stop them
 *
 *  Created: Fri Sep 26 21:06:37 2008
 *  Copyright  2008  Daniel Beck
 *             2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __GUI_UTILS_PLUGIN_TREE_VIEW_H_
#define __GUI_UTILS_PLUGIN_TREE_VIEW_H_

#include <netcomm/fawkes/client_handler.h>
#include <core/utils/lock_queue.h>
#include <gui_utils/connection_dispatcher.h>

#include <gtkmm.h>
#ifdef HAVE_GCONFMM
#  include <gconfmm.h>
#endif

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FawkesNetworkClient;
class FawkesNetworkMessage;

class PluginTreeView
: public Gtk::TreeView
{
 public:
  PluginTreeView();
  PluginTreeView(BaseObjectType* cobject,
                 const Glib::RefPtr<Gtk::Builder> builder);
  virtual ~PluginTreeView();

  void set_network_client(fawkes::FawkesNetworkClient* client);
  void set_gconf_prefix(Glib::ustring gconf_prefix);

 private:
  class PluginRecord : public Gtk::TreeModelColumnRecord
  {
   public:
    PluginRecord()
      {
	add(index);
	add(name);
	add(description);
	add(loaded);
      }

    Gtk::TreeModelColumn<int> index;           /**< an index */
    Gtk::TreeModelColumn<Glib::ustring> name;  /**< the name of the plugin */
    Gtk::TreeModelColumn<Glib::ustring> description;  /**< description of the plugin */
    Gtk::TreeModelColumn<bool> loaded;         /**< the loaded status of the plugin */
  };

  void ctor();
  void on_status_toggled(const Glib::ustring& path);
  void on_connected();
  void on_disconnected();
  void on_message_received(fawkes::FawkesNetworkMessage *msg);
  void on_id_clicked();
  void on_status_clicked();
  void on_name_clicked();
  void on_config_changed();

  void append_plugin_column();

 private:
  Glib::RefPtr<Gtk::ListStore> m_plugin_list;
#ifdef HAVE_GCONFMM
  Glib::RefPtr<Gnome::Conf::Client> __gconf;
#endif
  PluginRecord m_plugin_record;

  sigc::connection __gconf_connection;
  Glib::ustring    __gconf_prefix;

  fawkes::ConnectionDispatcher m_dispatcher;
};

} // end namespace fawkes

#endif /*  __GUI_UTILS_PLUGIN_TREE_VIEW_H_ */
