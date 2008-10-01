
/***************************************************************************
 *  config_edit_dialog.cpp - Edit config entries
 *
 *  Created: Wed Sep 24 15:44:46 2008
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

#include <tools/config_editor/config_edit_dialog.h>
#include <gui_utils/utils.h>

using namespace fawkes;

/** @class ConfigEditDialog tools/config_editor/config_edit_dialog.h
 * Dialog to edit a config value.
 *
 * @author Daniel Beck
 */

/** @var ConfigEditDialog::m_ent_value
 * An entry field to edit the config value.
 */

/** Constructor.
 * @param cobject pointer to base object type
 * @param ref_xml Glade XML file
 */
ConfigEditDialog::ConfigEditDialog( BaseObjectType* cobject,
				    const Glib::RefPtr<Gnome::Glade::Xml>& ref_xml )
  : Gtk::Dialog(cobject)
{
  m_ent_value = dynamic_cast<Gtk::Entry*>( get_widget(ref_xml, "entValueEdit") );
}

/** Initialize the dialog.
 * @param path config path
 * @param type type of config entry
 * @param value value of the config entry
 * @param is_default config entry in default config
 */
void
ConfigEditDialog::init( const Glib::ustring& path, const Glib::ustring& type,
			const Glib::ustring& value, bool is_default )
{
  set_title(path);
  m_ent_value->set_text(value);
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
  return m_ent_value->get_text();
}
