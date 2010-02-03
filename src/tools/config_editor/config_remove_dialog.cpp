
/***************************************************************************
 *  config_remove_dialog.cpp - Remove config entries
 *
 *  Created: Thu Sep 25 18:53:13 2008
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

#include <tools/config_editor/config_remove_dialog.h>

/** @class ConfigRemoveDialog "config_remove_dialog.h"
 * Dialog to remove a config entry
 *
 * @author Daniel Beck
 */

/** @var ConfigRemoveDialog::m_lbl_path
 * A Gtk::Label that presents the path to be deleted.
 */

/** @var ConfigRemoveDialog::m_chb_is_default
 * The Gtk::CheckButton to set the remove default flag
 */

/** Constructor.
 * @param lbl_path label of path to delete
 * @param chb_is_default checkbutton for default value deletion
 */
ConfigRemoveDialog::ConfigRemoveDialog(Gtk::Label *lbl_path, Gtk::CheckButton *chb_is_default)
{
  m_lbl_path = lbl_path;
  m_chb_is_default = chb_is_default;
}

#ifdef HAVE_GLADEMM
/** Constructor.
 * @param cobject pointer to base object type
 * @param ref_xml Glade XML file
 */
ConfigRemoveDialog::ConfigRemoveDialog( BaseObjectType* cobject,
					const Glib::RefPtr<Gnome::Glade::Xml>& ref_xml )
  : Gtk::Dialog(cobject)
{
  ref_xml->get_widget("lblPath", m_lbl_path);
  ref_xml->get_widget("chbIsDefaultRemove", m_chb_is_default);
}
#endif

/** Destructor. */
ConfigRemoveDialog::~ConfigRemoveDialog()
{
}

/** Initialize the dialog.
 * @param path the config path that was selected for deletion.
 * @param is_default true if only the default config value is set
 */
void
ConfigRemoveDialog::init(const Glib::ustring& path, bool is_default)
{
  set_title("Remove config entry");
  Glib::ustring text = "Really remove <b>" + path + "</b>?";
  m_lbl_path->set_markup(text);
  m_chb_is_default->set_active(is_default);
}

/** Get the remove default flag of the entry to be deleted
 * @return if true delete also the default config value
 */
bool
ConfigRemoveDialog::get_remove_default() const
{
  return m_chb_is_default->get_active();
}
