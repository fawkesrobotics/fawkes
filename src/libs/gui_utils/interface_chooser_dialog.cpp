
/***************************************************************************
 *  interface_chooser_dialog.cpp - Dialog for choosing a blackboard interface
 *
 *  Created: Sat Mar 19 12:21:40 2011
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

#include <gui_utils/interface_chooser_dialog.h>

#include <gtkmm.h>
#include <core/exception.h>
#include <core/exceptions/software.h>
#include <blackboard/blackboard.h>
#include <interface/interface_info.h>

#include <cstring>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Default title of interface chooser dialogs. */
const char* const InterfaceChooserDialog::DEFAULT_TITLE = "Select Interfaces";

/** @class InterfaceChooserDialog::Record <gui_utils/interface_chooser_dialog.h>
 * Blackboard interface record.
 * Record with information about a blackboard interface for a tree model.
 * @author Tim Niemueller
 */

/** Constructor. */
InterfaceChooserDialog::Record::Record()
{
  add(type);
  add(id);
  add(has_writer);
  add(num_readers);
}

/** @class InterfaceChooserDialog <gui_utils/interface_chooser_dialog.h>
 * Blackboard interface chooser dialog.
 * Allows to choose a blackboard interface from a list of interfaces matching
 * given type and ID patterns.
 * @author Tim Niemueller, Christoph Schwering
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
 * @param title title of the dialog
 * @return new InterfaceChooserDialog
 */
InterfaceChooserDialog*
InterfaceChooserDialog::create(
    Gtk::Window& parent,
    BlackBoard* blackboard,
    const char* type_pattern,
    const char* id_pattern,
    const Glib::ustring& title)
{
  InterfaceChooserDialog* d = new InterfaceChooserDialog(parent, title);
  d->init(blackboard, type_pattern, id_pattern);
  return d;
}


/** Constructor for subclasses.
 *
 * After calling this constructor, the init() method needs to be called.
 *
 * @param parent parent window
 * @param title title of the dialog
 */
InterfaceChooserDialog::InterfaceChooserDialog(Gtk::Window& parent,
					       const Glib::ustring& title)
  : Gtk::Dialog(title, parent, /* modal */ true),
    __parent(parent),
    __record(NULL)
{
  // May *NOT* call init(), because init() calls virtual methods.
}


/** Initialization method.
 *
 * Subclasses should use the protected constructor and should then call the
 * init() method.
 * This ensures that init()'s calls to virtual methods dispatch to the right
 * ones.
 *
 * @param blackboard blackboard instance to query interfaces from
 * @param type_pattern pattern with shell like globs (* for any number of
 * characters, ? for exactly one character) to match the interface type.
 * @param id_pattern pattern with shell like globs (* for any number of
 * characters, ? for exactly one character) to match the interface ID.
 */
void
InterfaceChooserDialog::init(BlackBoard *blackboard,
                             const char *type_pattern,
                             const char *id_pattern)
{
  __model = Gtk::ListStore::create(record());

  set_default_size(360, 240);

  __treeview.set_model(__model);
  init_columns();
  __scrollwin.add(__treeview);
  __scrollwin.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
  __treeview.show();

  Gtk::Box *vbox = get_vbox();
  vbox->pack_start(__scrollwin);
  __scrollwin.show();

  add_button(Gtk::Stock::CANCEL, 0);
  add_button(Gtk::Stock::OK, 1);

  set_default_response(1);

  __treeview.signal_row_activated().connect(sigc::bind(sigc::hide<0>(sigc::hide<0>(sigc::mem_fun(*this, &InterfaceChooserDialog::response))), 1));

  __bb = blackboard;

  InterfaceInfoList *infl = __bb->list(type_pattern, id_pattern);
  for (InterfaceInfoList::iterator i = infl->begin(); i != infl->end(); ++i) {
    Gtk::TreeModel::Row row = *__model->append();
    init_row(row, *i);
  }
  delete infl;
}


/** Destructor. */
InterfaceChooserDialog::~InterfaceChooserDialog()
{
  if (__record) {
    delete __record;
  }
}


/** Returns the Record of this chooser dialog.
 * Subclasses of InterfaceChooserDialog might want to override this method.
 * @return Record implementation.
 */
const InterfaceChooserDialog::Record&
InterfaceChooserDialog::record() const
{
  if (!__record) {
    InterfaceChooserDialog* this_nonconst = const_cast<InterfaceChooserDialog*>(this);
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
InterfaceChooserDialog::init_columns()
{
  __treeview.append_column("Type", record().type);
  __treeview.append_column("ID", record().id);
  __treeview.append_column("Writer?", record().has_writer);
  __treeview.append_column("Readers", record().num_readers);
  return 4;
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
InterfaceChooserDialog::init_row(Gtk::TreeModel::Row& row,
                                 const InterfaceInfo& ii)
{
  row[record().type]         = ii.type();
  row[record().id]           = ii.id();
  row[record().has_writer]   = ii.has_writer();
  row[record().num_readers]  = ii.num_readers();
}


/** Get selected interface type and ID.
 * If an interface has been selected use this method to get the
 * type and ID.
 * Only applicable if get_multi() == false.
 * @param type upon return contains the type of the interface
 * @param id upon return contains the ID of the interface
 * @exception Exception thrown if no interface has been selected
 */
void
InterfaceChooserDialog::get_selected_interface(Glib::ustring &type,
					       Glib::ustring &id)
{
  const Glib::RefPtr<Gtk::TreeSelection> treesel = __treeview.get_selection();
  const Gtk::TreeModel::iterator iter = treesel->get_selected();
  if (iter) {
    const Gtk::TreeModel::Row row = *iter;
    type   = row[record().type];
    id     = row[record().id];
  } else {
    throw Exception("No interface selected");
  }
}


/** Run dialog and try to connect.
 * This runs the service chooser dialog and connects to the given service
 * with the attached FawkesNetworkClient. If the connection couldn't be established
 * an error dialog is shown. You should not rely on the connection to be
 * active after calling this method, rather you should use a ConnectionDispatcher
 * to get the "connected" signal.
 * @return interface instant of the selected interface. Note that this is only
 * an untyped interface instance which is useful for instrospection purposes
 * only.
 */
fawkes::Interface *
InterfaceChooserDialog::run_and_open_for_reading()
{
  if (__bb->is_alive()) throw Exception("BlackBoard is not alive");

  if ( run() ) {
    try {
      Glib::ustring type;
      Glib::ustring id;

      return __bb->open_for_reading(type.c_str(), id.c_str());
    } catch (Exception &e) {
      Glib::ustring message = *(e.begin());
      Gtk::MessageDialog md(__parent, message, /* markup */ false,
			    Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			    /* modal */ true);
      md.set_title("Opening Interface failed");
      md.run();
      throw;
    }
  } else {
    return NULL;
  }
}

} // end of namespace fawkes
