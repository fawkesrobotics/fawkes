
/***************************************************************************
 *  config_add_dialog.cpp - Add config entries
 *
 *  Created: Thu Sep 25 17:31:40 2008
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

#include <tools/config_editor/config_add_dialog.h>

/** @class ConfigAddDialog "config_add_dialog.h"
 * Dialog to add a config entry
 *
 * @author Daniel Beck
 */

/** @var ConfigAddDialog::m_ent_path
 * The Gtk::Entry that contains the path of the new entry.
 */

/** @var ConfigAddDialog::m_ent_value
 * The Gtk::Entry that contains the value of the new entry.
 */

/** @var ConfigAddDialog::m_cob_bool_value
 * A combo box to select TRUE or FALSE
 */

/** @var ConfigAddDialog::m_type_pages
 * A Gtk::Notebook element to switch between boolean values and the rest
 */

/** @var ConfigAddDialog::m_cmb_type
 * The Gtk::ComboBox to select the type of the new entry.
 */

/** @var ConfigAddDialog::m_chb_is_default
 * The Gtk::CheckButton to set the default flag
 */

/** Constructor.
 * @param ent_path entry field for path
 * @param ent_value entry field for value
 * @param cob_bool_value combo box for bool values
 * @param type_pages pages for different types
 * @param cmb_type combo box for type
 * @param chb_is_default check button for default values
 */
ConfigAddDialog::ConfigAddDialog(Gtk::Entry       *ent_path,
				 Gtk::Entry       *ent_value,
				 Gtk::ComboBox    *cob_bool_value,
				 Gtk::Notebook    *type_pages,
				 Gtk::ComboBox    *cmb_type,
				 Gtk::CheckButton *chb_is_default)
{
  m_ent_path = ent_path;
  m_cmb_type = cmb_type;
  m_ent_value = ent_value;
  m_cob_bool_value = cob_bool_value;
  m_type_pages = type_pages;
  m_chb_is_default = chb_is_default;
  
  m_cmb_type->signal_changed().connect( sigc::mem_fun( *this, &ConfigAddDialog::on_my_changed) );
}

#ifdef HAVE_GLADEMM
/** Constructor.
 * @param cobject pointer to base object type
 * @param ref_xml Glade XML file
 */
ConfigAddDialog::ConfigAddDialog( BaseObjectType* cobject,
				  const Glib::RefPtr<Gnome::Glade::Xml>& ref_xml )
  : Gtk::Dialog(cobject)
{
  ref_xml->get_widget("entPathAdd", m_ent_path);
  ref_xml->get_widget("cmbTypeAdd", m_cmb_type);
  ref_xml->get_widget("entValueAdd", m_ent_value);
  ref_xml->get_widget("cmbBoolAdd", m_cob_bool_value);
  ref_xml->get_widget("nbkTypesAdd", m_type_pages);
  ref_xml->get_widget("chbIsDefaultAdd", m_chb_is_default);
  
  m_cmb_type->signal_changed().connect( sigc::mem_fun( *this, &ConfigAddDialog::on_my_changed) );
}
#endif

/** Destructor. */
ConfigAddDialog::~ConfigAddDialog()
{
}

/** Initialize the dialog.
 * @param path the config path of the selected row
 */
void
ConfigAddDialog::init(const Glib::ustring& path)
{
  m_ent_path->set_text(path);
  m_ent_value->set_text("");
  m_cmb_type->set_active(-1);
  m_cob_bool_value->set_active(-1);
  m_chb_is_default->set_active(true);
}

/** Get the path of the new entry.
 * @return the path of the new entry
 */
Glib::ustring
ConfigAddDialog::get_path() const
{
  return m_ent_path->get_text();
}

/** Get the type of the new entry.
 * @return the type of the new entry
 */
Glib::ustring
ConfigAddDialog::get_type() const
{
  Gtk::TreeIter iter = m_cmb_type->get_active();
  Gtk::TreeRow row = *iter;
  Glib::ustring type;  
  
  row.get_value(0, type);

  return type;
}

/** Get the value of the new entry.
 * @return the value of the new entry
 */
Glib::ustring
ConfigAddDialog::get_value() const
{
  if (get_type() != "bool") return m_ent_value->get_text();
  else
    {
      Gtk::TreeIter iter = m_cob_bool_value->get_active();
      Gtk::TreeRow row = *iter;
      Glib::ustring type;  
      
      row.get_value(0, type);
  
      return type;
    }
}

/** Get the default flag of the new entry
 * @return if true add to default config database
 */
bool
ConfigAddDialog::get_is_default() const
{
  return m_chb_is_default->get_active();
}

/**
 * Swiches the (invisible) pages to add either a bool or a different type value
 */
void
ConfigAddDialog::on_my_changed()
{
  m_type_pages->set_current_page(get_type() != "bool" ? 0 : 1);
}
