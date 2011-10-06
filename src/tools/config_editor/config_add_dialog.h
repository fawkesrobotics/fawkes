
/***************************************************************************
 *  config_add_dialog.h - Add config entries
 *
 *  Created: Thu Sep 25 15:59:43 2008
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

#ifndef __TOOLS_CONFIG_EDITOR_CONFIG_ADD_DIALOG_H_
#define __TOOLS_CONFIG_EDITOR_CONFIG_ADD_DIALOG_H_

#include <gtkmm.h>

class ConfigAddDialog : public Gtk::Dialog
{
 public:
  ConfigAddDialog(Gtk::Entry       *ent_path,
		  Gtk::Entry       *ent_value,
		  Gtk::ComboBox    *cob_bool_value,
		  Gtk::Notebook    *type_pages,
		  Gtk::ComboBox    *cmb_type,
		  Gtk::CheckButton *chb_is_default);
  ConfigAddDialog(BaseObjectType* cobject,
                  const Glib::RefPtr<Gtk::Builder> &builder);
  virtual ~ConfigAddDialog();

  void init(const Glib::ustring& path);

  Glib::ustring get_path() const;
  Glib::ustring get_type() const;
  Glib::ustring get_value() const;
  bool get_is_default() const;

 protected:
  void on_my_changed();
  
  Gtk::Entry       *m_ent_path;
  Gtk::Entry       *m_ent_value;
  Gtk::ComboBox    *m_cob_bool_value;
  Gtk::Notebook    *m_type_pages;
  Gtk::ComboBox    *m_cmb_type;
  Gtk::CheckButton *m_chb_is_default;
};

#endif /* __TOOLS_CONFIG_EDITOR_CONFIG_ADD_DIALOG_H_ */
