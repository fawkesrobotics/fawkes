
/***************************************************************************
 *  config_tree_view.cpp - TreeView class for displaying the configuration
 *
 *  Created: Wed Sep 24 13:45:39 2008
 *  Copyright  2008  Daniel Beck
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

#include <tools/config_editor/config_tree_view.h>
#include <tools/config_editor/config_edit_dialog.h>
#include <tools/config_editor/config_add_dialog.h>
#include <tools/config_editor/config_remove_dialog.h>
#include <config/netconf.h>
#include <config/sqlite.h>
#include <netcomm/fawkes/client.h>

#include <cstring>
#include <iostream>
#include <sstream>

using namespace std;
using namespace fawkes;

/** @class ConfigTreeView tools/config_editor/config_tree_view.h
 * Treeview widget for displaying/editing config entries.
 *
 * @author Daniel Beck
 */

/** @class ConfigTreeView::ConfigRecord tools/config_editor/config_tree_view.h
 * Column record class for the config TreeView.
 *
 * @author Daniel Beck
 */

/** @var ConfigTreeView::m_config_record
 * Column record object to access the columns of the storage object.
 */

/** @var ConfigTreeView::m_config_tree
 * Storage object.
 */

/** @var ConfigTreeView::m_menu
 * A popup menu to edit the configuration.
 */

/** @var ConfigTreeView::m_dlg_edit
 * A dialog to edit a config entry.
 */

/** @var ConfigTreeView::m_dlg_add
 * A dialog to add a config entry.
 */

/** @var ConfigTreeView::m_dlg_remove
 * A dialog to remove a config entry.
 */

/** @var ConfigTreeView::m_config
 * The fawkes::Configuration that is displayed and editted.
 */

/** @var ConfigTreeView::m_own_config
 * True if config was created by ConfigTreeView object.
 */

/** Constructor.
 * @param cobject pointer to base object type
 * @param ref_xml Glade XML file
 */
ConfigTreeView::ConfigTreeView( BaseObjectType* cobject,
				const Glib::RefPtr<Gnome::Glade::Xml>& ref_xml)
  : Gtk::TreeView(cobject)
{
  m_dlg_edit = NULL;
  ref_xml->get_widget_derived("dlgConfigEdit", m_dlg_edit);
  
  m_dlg_add = NULL;
  ref_xml->get_widget_derived("dlgConfigAdd", m_dlg_add);
  
  m_dlg_remove = NULL;
  ref_xml->get_widget_derived("dlgConfigRemove", m_dlg_remove);
  
  m_config_tree = Gtk::TreeStore::create(m_config_record);
  set_model(m_config_tree);
  append_column("Path", m_config_record.node);
  append_column("Value", m_config_record.value_string);

  Gtk::Menu::MenuList& menulist = m_menu.items();

  menulist.push_back( Gtk::Menu_Helpers::MenuElem("Edit", sigc::mem_fun( *this, &ConfigTreeView::on_menu_edit_selected) ) );
  menulist.push_back( Gtk::Menu_Helpers::MenuElem("Remove", sigc::mem_fun( *this, &ConfigTreeView::on_menu_remove_selected) ) );
  menulist.push_back( Gtk::Menu_Helpers::MenuElem("Add", sigc::mem_fun( *this, &ConfigTreeView::on_menu_add_selected) ) );

  m_config = NULL;
  m_own_config = false;

  signal_button_press_event().connect_notify( sigc::mem_fun(*this, &ConfigTreeView::on_button_press_event_custom) );
}

/** Destructor. */
ConfigTreeView::~ConfigTreeView()
{
  if (m_own_config)
    { delete m_config; }
}

/** Set the fawkes::Configuration to be displayed.
 * @param config the fawkes::Configuration
 */
void
ConfigTreeView::set_config(Configuration* config)
{
  m_config = config;
  m_own_config = false;

  // TODO: enable mirror mode if it is a netconf

  read_config();
}

/** Set a network client that is used to open a fawkes::NetworkConfiguration.
 * @param client a fawkes::NetworkClient
 */
void
ConfigTreeView::set_network_client(FawkesNetworkClient* client)
{
  NetworkConfiguration* netconf = new NetworkConfiguration(client);
  netconf->set_mirror_mode(true);
  m_config = netconf;
  m_own_config = true;

  read_config();
}

/** Set the file to read the config from.
 * @param filename the filename of the database file
 */
void
ConfigTreeView::set_config_file(const char* filename)
{
  m_config = new SQLiteConfiguration(filename);
  m_own_config = true;

  read_config();
}

void
ConfigTreeView::read_config()
{
  if ( !m_config )
    { return; }

  m_config_tree->clear();

  m_config->lock();
  Configuration::ValueIterator* cit = m_config->iterator();
  while ( cit->next() )
    {
      if ( cit->is_bool() )
	{ set_value(cit->path(), cit->type(), cit->is_default(), cit->get_bool()); }
      else if ( cit->is_int() )
	{ set_value(cit->path(), cit->type(), cit->is_default(), cit->get_int()); }
      else if ( cit->is_uint() )
	{ set_value(cit->path(), cit->type(), cit->is_default(), cit->get_uint()); }
      else if ( cit->is_float() )
	{ set_value(cit->path(), cit->type(), cit->is_default(), cit->get_float()); }
      else if ( cit->is_string() )
	{ set_value(cit->path(), cit->type(), cit->is_default(), cit->get_string()); }
    }
  
  delete cit;
  m_config->unlock();
}


/** Add a config entry to the TreeModel storage object.
 * @param path config path
 * @param type type of config entry
 * @param is_default true if config entry is in the default config
 * @param value the value of the config entry
 */
void
ConfigTreeView::set_value(const char* path, const char* type, bool is_default, bool value)
{
  Gtk::TreeModel::Row row;
  row = *get_iter(path);

  row[m_config_record.path] = path;
  row[m_config_record.type] = type;
  row[m_config_record.is_default] = is_default;
  row[m_config_record.value_bool] = value;
  row[m_config_record.value_string] = ( value ? "TRUE" : "FALSE" );
}

/** Add a config entry to the TreeModel storage object.
 * @param path config path
 * @param type type of config entry
 * @param is_default true if config entry is in the default config
 * @param value the value of the config entry
 */
void
ConfigTreeView::set_value(const char* path, const char* type, bool is_default, int value)
{
  Gtk::TreeModel::Row row;
  row = *get_iter(path);

  row[m_config_record.path] = path;
  row[m_config_record.type] = type;
  row[m_config_record.is_default] = is_default;
  row[m_config_record.value_int] = value;

  string val_str;
  stringstream ss;
  ss << value;
  ss >> val_str;
  row[m_config_record.value_string] = val_str;
}

/** Add a config entry to the TreeModel storage object.
 * @param path config path
 * @param type type of config entry
 * @param is_default true if config entry is in the default config
 * @param value the value of the config entry
 */
void
ConfigTreeView::set_value(const char* path, const char* type, bool is_default, uint value)
{
  Gtk::TreeModel::Row row;
  row = *get_iter(path);

  row[m_config_record.path] = path;
  row[m_config_record.type] = type;
  row[m_config_record.is_default] = is_default;
  row[m_config_record.value_uint] = value;

  string val_str;
  stringstream ss;
  ss << value;
  ss >> val_str;
  row[m_config_record.value_string] = val_str;
}

/** Add a config entry to the TreeModel storage object.
 * @param path config path
 * @param type type of config entry
 * @param is_default true if config entry is in the default config
 * @param value the value of the config entry
 */
void
ConfigTreeView::set_value(const char* path, const char* type, bool is_default, float value)
{
  Gtk::TreeModel::Row row;
  row = *get_iter(path);

  row[m_config_record.path] = path;
  row[m_config_record.type] = type;
  row[m_config_record.is_default] = is_default;
  row[m_config_record.value_float] = value;

  string val_str;
  stringstream ss;
  ss << value;
  ss >> val_str;
  row[m_config_record.value_string] = val_str;
}

/** Add a config entry to the TreeModel storage object.
 * @param path config path
 * @param type type of config entry
 * @param is_default true if config entry is in the default config
 * @param value the value of the config entry
 */
void
ConfigTreeView::set_value(const char* path, const char* type, bool is_default, std::string value)
{
  Gtk::TreeModel::Row row;
  row = *get_iter(path);

  row[m_config_record.path] = path;
  row[m_config_record.type] = type;
  row[m_config_record.is_default] = is_default;
  row[m_config_record.value_string] = value;
}

Gtk::TreeIter
ConfigTreeView::get_iter(const char* p)
{
  char* path;
  char* full_path;

  asprintf(&full_path, p);
  char* node = strtok(full_path, "/");

  asprintf(&path, "/%s/", node);
  
  Gtk::TreeModel::Children children = m_config_tree->children();
  Gtk::TreeIter iter = children.begin();

  while ( node != NULL )
    {
      bool found = false;
      iter = children.begin();
      
      while ( !found && iter != children.end() )
	{
	  Gtk::TreeModel::Row row = *iter;
	  
	  Glib::ustring r = row[m_config_record.node];
	  if ( strcmp(r.c_str(), node) == 0 )
	    {
	      found = true;
	      children = row.children();
	      iter = children.begin();
	    }
	  else
	    { ++iter; }
	}
      
      if ( !found )
	{
	  iter = m_config_tree->append(children);
	  Gtk::TreeModel::Row row = *iter;
	  row[m_config_record.node] = Glib::ustring(node);
	  row[m_config_record.path] = Glib::ustring(path);
	  
	  children = row.children();
	}
      
      node = strtok(NULL, "/");

      char* t;
      asprintf(&t, "%s/%s/", path, node);
      free(path);
      path = t;
    }
  
  free(path);
  free(full_path);

  return iter;
}

/** Signal handler for the button press event.
 * @param event a Gdk button event
 * @return true if signal has been handled, false otherwise
 */
void
ConfigTreeView::on_button_press_event_custom(GdkEventButton* event)
{
  if (event->type == GDK_2BUTTON_PRESS)
    {
      edit_entry( get_selection()->get_selected() );
    }
  else if ( event->type == GDK_BUTTON_PRESS && (event->button == 3) )
    {
      m_menu.popup(event->button, event->time);
    }
}

/** Signal handler that is called when the 'edit' entry is selected
 * from popup menu.
 */
void ConfigTreeView::on_menu_edit_selected() 
{
  edit_entry( get_selection()->get_selected() );
}

/** Signal handler that is called when the 'add' entry is selected
 * from popup menu.
 */
void
ConfigTreeView::on_menu_add_selected()
{
  add_entry( get_selection()->get_selected() );
}

/** Signal handler that is called when the 'remove' entry is selected
 * from popup menu.
 */
void
ConfigTreeView::on_menu_remove_selected()
{
  remove_entry( get_selection()->get_selected() );
}

bool
ConfigTreeView::edit_entry(const Gtk::TreeIter& iter)
{
  bool ret_val;

  Gtk::TreeModel::Row row = *iter;
  Glib::ustring value = row[m_config_record.value_string];
  
  if (value == "")
    { ret_val = false; }
  else
    {
      int result;
      Glib::ustring path = row[m_config_record.path];
      Glib::ustring type = row[m_config_record.type];
      bool is_default    = row[m_config_record.is_default];

      m_dlg_edit->init(path, type, value, is_default);
      Gtk::Window* parent = dynamic_cast<Gtk::Window*>( get_toplevel() );
      m_dlg_edit->set_transient_for(*parent);
      result = m_dlg_edit->run();

      switch (result)
	{
	case Gtk::RESPONSE_OK:
	  {
	    Glib::ustring value = m_dlg_edit->get_value();

	    const char* p = path.c_str();
	    const char* t = type.c_str();

	    if ( m_config->is_bool(p) ) 
	      {
		bool b;
		if (value == "TRUE")
		  { b = true; }
		else if (value == "FALSE")
		  { b = false; }
		
		m_config->set_bool(p, b);
 		set_value(p, t, is_default, b);
	      }
	    else if ( m_config->is_int(p) ) 
	      {
		int i;
		i = atoi( value.c_str() );
		
		m_config->set_int(p, i);
		set_value(p, t, is_default, i);
	      }
	    else if ( m_config->is_uint(p) )
	      {
		int i;
		i = atoi( value.c_str() );
		if ( 0 <= i)
		  { 
		    m_config->set_uint( p, (unsigned int) i );
		    set_value(p, t, is_default, (unsigned int) i);
		  }
	      }
	    else if ( m_config->is_float(p) ) 
	      {
		float f;
		f = atof( value.c_str() );
		
		m_config->set_float(p, f);
		set_value(p, t, is_default, f);
	      }
	    else if ( m_config->is_string(p) ) 
	      {
		string s( value.c_str() );
		
		m_config->set_string(p, s);
		set_value(p, t, is_default, s);
	      }

	    ret_val = true;

	    break;
	  }

	default:
	  ret_val = false;
	  break;
	}

      m_dlg_edit->hide();
    }

  return ret_val;
}

bool
ConfigTreeView::remove_entry(const Gtk::TreeIter& iter)
{
  bool ret_val = false;
  int result;
  Gtk::TreeModel::Row row = *iter;
  Glib::ustring value = row[m_config_record.value_string];

  if (value == "")
    { ret_val = false; }
  else
    {
      Glib::ustring path = row[m_config_record.path];
      m_dlg_remove->init(path);
      
      Gtk::Window* parent = dynamic_cast<Gtk::Window*>( get_toplevel() );
      m_dlg_remove->set_transient_for(*parent);
      result = m_dlg_remove->run();
      
      switch (result)
	{
	case Gtk::RESPONSE_OK:
	  {
	    const char* p = path.c_str();
	    m_config->erase(p);

// 	    cout << "deleting: " 
// 		 << row[m_config_record.m_path] 
// 		 << "/"
// 		 << row[m_config_record.m_value_string]
// 		 << endl;

	    Gtk::TreePath tree_path = m_config_tree->get_path(iter);
  	    m_config_tree->erase(iter);
	    m_config_tree->row_deleted(tree_path);

	    break;
	  }
	  
	default:
	  ret_val = false;
	  break;
	}

      m_dlg_remove->hide();
    }

  return ret_val;
}

bool
ConfigTreeView::add_entry(const Gtk::TreeIter& iter)
{
  bool ret_val = false;
  int result;
  Gtk::TreeModel::Row row = *iter;
  Glib::ustring path = row[m_config_record.path];

  m_dlg_add->init(path);

  Gtk::Window* parent = dynamic_cast<Gtk::Window*>( get_toplevel() );
  m_dlg_add->set_transient_for(*parent);
  result = m_dlg_add->run();

  switch (result)
    {
    case Gtk::RESPONSE_OK:
      {
	Glib::ustring type = m_dlg_add->get_type();
	Glib::ustring path = m_dlg_add->get_path();
	Glib::ustring value = m_dlg_add->get_value();

	const char* t = type.c_str();
	const char* p = path.c_str();

	bool is_default = m_dlg_add->get_is_default();
	
	ret_val = true;

	if ( type == "bool" )
	  {
	    bool b;

	    if ( value == "TRUE" || value == "true" )
	      { b = true; }
	    else if ( value == "FALSE" || value == "false" )
	      { b = false; }

	    m_config->set_bool(p, b);
	    set_value(p, t, is_default, b);
	  }

	else if ( type == "int" )
	  {
	    int i;
	    i = atoi( value.c_str() );
	    
	    m_config->set_int(p, i);
	    set_value(p, t, is_default, i);
	  }

	else if ( type == "uint" )
	  {
	    int i;
	    i = atoi( value.c_str() );
	    if ( 0 <= i)
	      { 
		m_config->set_uint( p, (unsigned int) i );
		set_value(p, t, is_default, (unsigned int) i);
	      }
	  }

	else if ( type == "float" )
	  {
	    float f;
	    f = atof( value.c_str() );
	    
	    m_config->set_float(p, f);
	    set_value(p, t, is_default, f);
	  }

	else if ( type == "string")
	  {
	    string s( value.c_str() );
	    
	    m_config->set_string(p, s);
	    set_value(p, t, is_default, s);
	  }

	else
	  {
	    ret_val = false;
	    cout << "Unknown type." << endl;
	  }
	
	break;
      }
      
    default:
      ret_val = false;
      break;
    }

  m_dlg_add->hide();

  return ret_val;
}
