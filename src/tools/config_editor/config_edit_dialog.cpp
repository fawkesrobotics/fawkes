
/***************************************************************************
 *  config_edit_dialog.cpp - Edit config entries
 *
 *  Created: Wed Sep 24 15:44:46 2008
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

#include <tools/config_editor/config_edit_dialog.h>

/** @class ConfigEditDialog "config_edit_dialog.h"
 * Dialog to edit a config value.
 *
 * @author Daniel Beck
 */

/** @var ConfigEditDialog::is_bool
 * A flag to store wether the config value is boolean.
 */

/** @var ConfigEditDialog::m_ent_value
 * An entry field to edit the config value.
 */

/** @var ConfigEditDialog::m_cob_bool_value
 * A combo box to select TRUE or FALSE
 */

/** @var ConfigEditDialog::m_type_pages
 * A Gtk::Notebook element to switch between boolean values and the rest
 */

/** @var ConfigEditDialog::m_chb_is_default
 * The Gtk::CheckButton to set the default flag
 */

/** Constructor.
 * @param ent_value entry field for value
 * @param cob_bool_value combo box for bool value
 * @param type_pages pages for types
 * @param chb_is_default checkbutton to mark default values
 */     
ConfigEditDialog::ConfigEditDialog(Gtk::Entry *ent_value, Gtk::ComboBox *cob_bool_value,
				   Gtk::Notebook *type_pages, Gtk::CheckButton *chb_is_default)
  : Gtk::Dialog()
{
  m_ent_value       = m_ent_value;
  m_cob_bool_value  = cob_bool_value;
  m_type_pages      = type_pages;
  m_chb_is_default  = chb_is_default;
}

/** Constructor.
 * @param cobject pointer to base object type
 * @param builder Gtk builder
 */     
ConfigEditDialog::ConfigEditDialog(BaseObjectType* cobject,
                                   const Glib::RefPtr<Gtk::Builder> &builder)
  : Gtk::Dialog(cobject)
{
  builder->get_widget("entValueEdit", m_ent_value);
  builder->get_widget("cmbBoolEdit", m_cob_bool_value);
  builder->get_widget("nbkTypesEdit", m_type_pages);
  builder->get_widget("chbIsDefaultEdit", m_chb_is_default);
}

/** Initialize the dialog.
 * @param path config path
 * @param type type of config entry
 * @param value value of the config entry
 */
void
ConfigEditDialog::init( const Glib::ustring& path, const Glib::ustring& type,
			const Glib::ustring& value )
{
  is_bool = (type == "bool");
  set_title(path);
  m_ent_value->set_text(value);
  m_cob_bool_value->set_active(value == "TRUE" ? 0 : 1);
  m_type_pages->set_current_page(!is_bool ? 0 : 1);
  m_chb_is_default->set_active(false);
}

/** Destructor. */
ConfigEditDialog::~ConfigEditDialog()
{
}

/** Get the value of the entry widget.
 * @return the text in the entry widget
 */
Glib::ustring
ConfigEditDialog::get_value() const
{
  if (!is_bool) return m_ent_value->get_text();
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
 * @return if true edit the default config database
 */
bool
ConfigEditDialog::get_is_default() const
{
  return m_chb_is_default->get_active();
}
