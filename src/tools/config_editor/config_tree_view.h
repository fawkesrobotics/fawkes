
/***************************************************************************
 *  config_tree_view.h - TreeView class for displaying the configuration
 *
 *  Created: Wed Sep 24 13:39:47 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __TOOLS_CONFIG_EDITOR_CONFIG_TREE_VIEW_H_
#define __TOOLS_CONFIG_EDITOR_CONFIG_TREE_VIEW_H_

#include <gtkmm.h>
#include <string>

namespace fawkes {
  class Configuration;
  class FawkesNetworkClient;
}

class ConfigEditDialog;
class ConfigAddDialog;
class ConfigRemoveDialog;
class ConfigEditorPlugin;

class ConfigTreeView : public Gtk::TreeView
{
 public:
  ConfigTreeView(BaseObjectType* cobject,
                 const Glib::RefPtr<Gtk::Builder>& builder);
  virtual ~ConfigTreeView();

  void set_config(fawkes::Configuration* config);
  void set_network_client(fawkes::FawkesNetworkClient* client);
  void set_config_file(const char* filename);

  void register_plugin( ConfigEditorPlugin* plugin );
  void remove_plugin( std::string config_path );
  
 protected:
  void set_value(const char* path, const char* type, bool is_default, bool value);
  void set_value(const char* path, const char* type, bool is_default, int value);
  void set_value(const char* path, const char* type, bool is_default, uint value);
  void set_value(const char* path, const char* type, bool is_default, float value);
  void set_value(const char* path, const char* type, bool is_default, std::string value);

  virtual void on_button_press_event_custom(GdkEventButton* event);
  virtual void on_menu_edit_selected();
  virtual void on_menu_add_selected();
  virtual void on_menu_remove_selected();
  
  class ConfigRecord : public Gtk::TreeModelColumnRecord
    {
    public:
      ConfigRecord()
	{
	  add(node);
	  add(path);
	  add(type);
	  add(is_default);
	  add(value_bool);
	  add(value_int);
	  add(value_uint);
	  add(value_float);
	  add(value_string);
	}
      
      Gtk::TreeModelColumn<Glib::ustring> node;          /**< node name */
      Gtk::TreeModelColumn<Glib::ustring> path;          /**< config path */
      Gtk::TreeModelColumn<Glib::ustring> type;          /**< config value type */
      Gtk::TreeModelColumn<bool>          is_default;    /**< default flag */
      Gtk::TreeModelColumn<bool>          value_bool;    /**< bool config value */
      Gtk::TreeModelColumn<int>           value_int;     /**< int config value */
      Gtk::TreeModelColumn<uint>          value_uint;    /**< unsigned int config value */
      Gtk::TreeModelColumn<float>         value_float;   /**< float config value */
      Gtk::TreeModelColumn<Glib::ustring> value_string;  /**< config value as string */
    };
  
  ConfigRecord m_config_record;
  Glib::RefPtr<Gtk::TreeStore> m_config_tree;

  Gtk::Menu m_menu;
  ConfigEditDialog* m_dlg_edit;
  ConfigAddDialog* m_dlg_add;
  ConfigRemoveDialog* m_dlg_remove;

  std::map< std::string, ConfigEditorPlugin* > m_plugins;

  fawkes::Configuration* m_config;
  bool m_own_config;

 private:
  void read_config();

  Gtk::TreeIter get_iter(const char* path);
  Gtk::TreeIter search_path( const char* path );

  bool edit_entry(const Gtk::TreeIter& iter);
  bool add_entry(const Gtk::TreeIter& iter);
  bool remove_entry(const Gtk::TreeIter& iter);
};

#endif /* __TOOLS_CONFIG_EDITOR_CONFIG_TREE_VIEW_H_ */
