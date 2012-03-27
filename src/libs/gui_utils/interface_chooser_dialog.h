/***************************************************************************
 *  interface_chooser_dialog.h - Dialog for choosing a blackboard interface
 *
 *  Created: Sat Mar 19 12:18:43 2011
 *  Copyright  2008-2011  Tim Niemueller [www.niemueller.de]
 *  Copyright  2011       Christoph Schwering
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

#ifndef __LIBS_GUI_UTILS_INTERFACE_CHOOSER_DIALOG_H_
#define __LIBS_GUI_UTILS_INTERFACE_CHOOSER_DIALOG_H_

#include <gtkmm/dialog.h>
#include <gtkmm/treeview.h>
#include <gtkmm/entry.h>
#include <gtkmm/expander.h>
#include <gtkmm/scrolledwindow.h>
#include <gtkmm/liststore.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Interface;
class BlackBoard;
class InterfaceInfo;

class InterfaceChooserDialog
  : public Gtk::Dialog
{
 public:
  static const char* const DEFAULT_TITLE;

  static InterfaceChooserDialog* create(
      Gtk::Window &parent,
      BlackBoard *blackboard,
      const char *type_pattern,
      const char *id_pattern,
      const Glib::ustring& title = DEFAULT_TITLE);

  virtual ~InterfaceChooserDialog();

  void get_selected_interface(Glib::ustring &type, Glib::ustring &id);

  fawkes::Interface *   run_and_open_for_reading();

 protected:
  class Record : public Gtk::TreeModelColumnRecord
  {
   public:
    Record();
      
    Gtk::TreeModelColumn<Glib::ustring> type;	/**< The type of the interface */
    Gtk::TreeModelColumn<Glib::ustring> id;	/**< The ID of the interface */
    Gtk::TreeModelColumn<bool> has_writer;	/**< Writer exists? */
    Gtk::TreeModelColumn<unsigned int> num_readers;	/**< Number of readers */
  };

  InterfaceChooserDialog(Gtk::Window& parent, const Glib::ustring& title);

  void init(BlackBoard* blackboard,
            const char* type_pattern,
            const char* id_pattern);

  virtual const Record& record() const;
  virtual int init_columns();
  virtual void init_row(Gtk::TreeModel::Row& row, const InterfaceInfo& ii);

  Gtk::TreeView                 __treeview; /**< Tree widget for interfaces. */
  Glib::RefPtr<Gtk::ListStore>  __model;    /**< Data model of the tree. */

 private:
  InterfaceChooserDialog(const InterfaceChooserDialog& obj);
  InterfaceChooserDialog& operator=(const InterfaceChooserDialog& obj);

  BlackBoard *__bb;

  Gtk::Window         &__parent;
  Gtk::ScrolledWindow  __scrollwin;

  const Record* __record; /**< Should only be accessed by record(). */
};

} // end of namespace fawkes

#endif
