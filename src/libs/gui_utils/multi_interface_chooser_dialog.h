/***************************************************************************
 *  multi_interface_chooser_dialog.h - Dialog for choosing a blackboard interface
 *
 *  Created: Mon Oct 17 21:01:30 2011
 *  Copyright  2011  Christoph Schwering
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __LIBS_GUI_UTILS_MULTI_INTERFACE_CHOOSER_DIALOG_H_
#define __LIBS_GUI_UTILS_MULTI_INTERFACE_CHOOSER_DIALOG_H_

#include <set>
#include <string>

#include <gui_utils/interface_chooser_dialog.h>

namespace fawkes {

class MultiInterfaceChooserDialog : public InterfaceChooserDialog
{
 public:
  /** Set of type and IDs of interfaces. */
  typedef std::pair<Glib::ustring, Glib::ustring> TypeIdPair;
  /** Pair of type and ID of an interface. */
  typedef std::set<TypeIdPair> TypeIdPairSet;

  static MultiInterfaceChooserDialog* create(
      Gtk::Window &parent,
      BlackBoard *blackboard,
      const char *type_pattern,
      const char *id_pattern,
      const TypeIdPairSet& loaded_interfaces,
      const Glib::ustring& title = DEFAULT_TITLE);

  virtual ~MultiInterfaceChooserDialog();

  TypeIdPairSet get_selected_interfaces() const;
  TypeIdPairSet get_newly_selected_interfaces() const;

 protected:
  class Record : public InterfaceChooserDialog::Record
  {
   public:
    Record();
    Gtk::TreeModelColumn<bool> load; /**< Load this interface? */
  };

  MultiInterfaceChooserDialog(Gtk::Window &parent,
                              const TypeIdPairSet& loaded_interfaces,
                              const Glib::ustring& title);

  virtual const Record& record() const;
  virtual int init_columns();
  virtual void init_row(Gtk::TreeModel::Row& row, const InterfaceInfo& ii);

 private:
  void on_load_toggled(const Glib::ustring& path);

  const Record* __record; /**< Should only be accessed by record(). */
  TypeIdPairSet __loaded_interfaces;
};

} // end of namespace fawkes

#endif
