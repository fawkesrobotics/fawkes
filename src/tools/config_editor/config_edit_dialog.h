
/***************************************************************************
 *  config_edit_dialog.h - Edit config entries
 *
 *  Created: Wed Sep 24 15:41:27 2008
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

#ifndef __TOOLS_CONFIG_EDITOR_CONFIG_EDIT_DIALOG_H_
#define __TOOLS_CONFIG_EDITOR_CONFIG_EDIT_DIALOG_H_

#include <gtkmm.h>
#include <libglademm/xml.h>

class ConfigEditDialog : public Gtk::Dialog
{
 public:
  ConfigEditDialog(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml>& ref_xml);
  virtual ~ConfigEditDialog();

  void init( const Glib::ustring& path, const Glib::ustring& type,
	     const Glib::ustring& value );

  Glib::ustring get_value() const;
  bool get_is_default() const;

 protected:
  bool              is_bool;
  Gtk::Entry       *m_ent_value;
  Gtk::ComboBox    *m_cob_bool_value;
  Gtk::Notebook    *m_type_pages;
  Gtk::CheckButton *m_chb_is_default;
};

#endif /* __TOOLS_CONFIG_EDITOR_CONFIG_EDIT_DIALOG_H_ */
