/***************************************************************************
 *  multi_interface_chooser_dialog.cpp - Dialog for choosing a blackboard interface
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

#include <gui_utils/multi_interface_chooser_dialog.h>

#include <gtkmm.h>
#include <core/exception.h>
#include <core/exceptions/software.h>
#include <blackboard/blackboard.h>
#include <interface/interface_info.h>

#include <algorithm>
#include <cassert>
#include <cstring>

namespace fawkes {

/** @class MultiInterfaceChooserDialog::Record <gui_utils/multi_interface_chooser_dialog.h>
 * Blackboard interface record.
 * Adds a checkbox whether or not to load the specific interface.
 * @author Christoph Schwering
 */

/** Constructor. */
MultiInterfaceChooserDialog::Record::Record()
{
  add(load);
}

/** @class MultiInterfaceChooserDialog <gui_utils/multi_interface_chooser_dialog.h>
 * Blackboard interface chooser dialog that supports multiple choices.
 * Allows to choose multiple blackboard interfaces from a list of interfaces
 * matching given type and ID patterns.
 * @author Christoph Schwering
 */


/** Factory method.
 *
 * Why a factory method instead of a ctor?
 * The factory method calls init(), and init() calls other virtual methods.
 * If this was a ctor, this ctor would not be allowed to be called by
 * subclasses, because then the virtual methods in init() don't dispatch the
 * right way during construction (see Effective C++ #9).
 *
 * @param parent parent window
 * @param blackboard blackboard instance to query interfaces from
 * @param type_pattern pattern with shell like globs (* for any number of
 * characters, ? for exactly one character) to match the interface type.
 * @param id_pattern pattern with shell like globs (* for any number of
 * characters, ? for exactly one character) to match the interface ID.
 * @param loaded_interfaces list of interfaces which are already loaded
 * @param title title of the dialog
 * @return new MultiInterfaceChooserDialog
 */
MultiInterfaceChooserDialog*
MultiInterfaceChooserDialog::create(
    Gtk::Window& parent,
    BlackBoard* blackboard,
    const char* type_pattern,
    const char* id_pattern,
    const TypeIdPairList& loaded_interfaces,
    const Glib::ustring& title)
{
  MultiInterfaceChooserDialog* d = new MultiInterfaceChooserDialog(
      parent, loaded_interfaces, title);
  d->init(blackboard, type_pattern, id_pattern);
  return d;
}


/** Constructor for subclasses.
 *
 * After calling this constructor, the init() method needs to be called.
 *
 * @param parent parent window
 * @param loaded_interfaces list of interfaces which are already loaded
 * @param title title of the dialog
 */
MultiInterfaceChooserDialog::MultiInterfaceChooserDialog(
    Gtk::Window &parent,
    const TypeIdPairList& loaded_interfaces,
    const Glib::ustring& title)
  : InterfaceChooserDialog(parent, title),
    __record(NULL)
{
  __loaded_interfaces.insert(loaded_interfaces.begin(), loaded_interfaces.end());
  Glib::RefPtr<Gtk::TreeSelection> treesel = __treeview.get_selection();
  __treeview.set_reorderable(true);
  __treeview.set_tooltip_text("Drag the rows to change the painting order.");
  treesel->set_mode(Gtk::SELECTION_NONE);
  // May *NOT* call init(), because init() calls virtual methods.
}


/** Destructor. */
MultiInterfaceChooserDialog::~MultiInterfaceChooserDialog()
{
  if (__record) {
    delete __record;
  }
}


void
MultiInterfaceChooserDialog::on_load_toggled(const Glib::ustring& path)
{
  Gtk::TreeModel::Row row = *__model->get_iter(path);
  row[record().load] = !row[record().load];
}


/** Returns the Record of this chooser dialog.
 * Subclasses of InterfaceChooserDialog might want to override this method.
 * @return Record implementation.
 */
const MultiInterfaceChooserDialog::Record&
MultiInterfaceChooserDialog::record() const
{
  if (!__record) {
    MultiInterfaceChooserDialog* this_nonconst = const_cast<MultiInterfaceChooserDialog*>(this);
    this_nonconst->__record = new Record();
  }
  return *__record;
}


/** Initializes the columns GUI-wise.
 * Called in the ctor.
 * Subclasses of InterfaceChooserDialog might want to override this method,
 * but should probably still call their super-class's implementation
 * (i.e., this one).
 * @return The number of columns added.
 */
int
MultiInterfaceChooserDialog::init_columns()
{
  __treeview.append_column("Load", record().load);

  const int n = InterfaceChooserDialog::init_columns();

  Gtk::CellRendererToggle* renderer = dynamic_cast<Gtk::CellRendererToggle*>(
      __treeview.get_column_cell_renderer(0));
  assert(renderer != NULL);

  renderer->set_activatable(true);
  renderer->signal_toggled().connect(
      sigc::mem_fun(*this, &MultiInterfaceChooserDialog::on_load_toggled));

  return n + 2;
}


/** Initializes a row with the given interface.
 * Called in the ctor.
 * Subclasses of InterfaceChooserDialog might want to override this method,
 * but should probably still call their super-class's implementation
 * (i.e., this one).
 * @param row The row whose content is to be set.
 * @param ii The interface info that should populate the row.
 */
void
MultiInterfaceChooserDialog::init_row(Gtk::TreeModel::Row& row,
                                      const InterfaceInfo& ii)
{
  InterfaceChooserDialog::init_row(row, ii);
  row[record().load] = __loaded_interfaces.find(std::make_pair(ii.type(), ii.id())) !=
                       __loaded_interfaces.end();
}


/** Get selected interface types and their respective IDs.
 * @return A list of type + id pairs of interfaces that are to be loaded.
 */
MultiInterfaceChooserDialog::TypeIdPairList
MultiInterfaceChooserDialog::get_selected_interfaces() const
{
  TypeIdPairList types_and_ids;

  const Gtk::TreeNodeChildren children = __model->children();
  for (Gtk::TreeNodeChildren::const_iterator it = children.begin();
       it != children.end(); ++it)
  {
    const Gtk::TreeRow& row = *it;
    if (row[record().load]) {
      TypeIdPair pair = std::make_pair(row[record().type], row[record().id]);
      types_and_ids.push_back(pair);
    }
  }

  return types_and_ids;
}


/** Get selected interface types and their respective IDs.
 * @return A list of type + id pairs of interfaces that are to be loaded,
 *         and *NOT* contained in the list of loaded interfaces handed
 *         over to create().
 */
MultiInterfaceChooserDialog::TypeIdPairList
MultiInterfaceChooserDialog::get_newly_selected_interfaces() const
{
  TypeIdPairList types_and_ids;

  const Gtk::TreeNodeChildren children = __model->children();
  for (Gtk::TreeNodeChildren::const_iterator it = children.begin();
       it != children.end(); ++it)
  {
    const Gtk::TreeRow& row = *it;
    if (row[record().load]) {
      TypeIdPair pair = std::make_pair(row[record().type], row[record().id]);
      if (__loaded_interfaces.find(pair) == __loaded_interfaces.end())
      {
        types_and_ids.push_back(pair);
      }
    }
  }

  return types_and_ids;
}

} // end of namespace fawkes

