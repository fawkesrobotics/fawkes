
/***************************************************************************
 *  service_chooser_dialog.cpp - Dialog for choosing a network service
 *
 *  Created: Sun Oct 12 17:06:06 2008 (split from lasergui_hildon.cpp)
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_GUI_UTILS_SERVICE_CHOOSER_DIALOG_H_
#define __LIBS_GUI_UTILS_SERVICE_CHOOSER_DIALOG_H_

#include <gui_utils/service_model.h>

#include <gtkmm/dialog.h>
#include <gtkmm/treeview.h>
#include <gtkmm/entry.h>
#include <gtkmm/expander.h>
#include <gtkmm/scrolledwindow.h>
#ifdef HAVE_GCONFMM
#  include <gconfmm.h>
#endif

#include <sys/types.h>
#include <sys/socket.h>

namespace fawkes {

class FawkesNetworkClient;
class ServiceModel;

class ServiceChooserDialog
  : public Gtk::Dialog
{
 public:
  ServiceChooserDialog(Gtk::Window &parent,
		       FawkesNetworkClient *client,
		       Glib::ustring title = "Select Service",
		       const char *service = "_fawkes._tcp");

  ServiceChooserDialog(Gtk::Window &parent,
		       Glib::ustring title = "Select Service",
		       const char *service = "_fawkes._tcp");

  virtual ~ServiceChooserDialog();

  void get_selected_service(Glib::ustring &name, Glib::ustring &hostname,
                            unsigned short int &port);
  void get_selected_service(Glib::ustring &hostname, struct sockaddr_storage &sockaddr);
  
  void get_raw_address(struct sockaddr *addr, socklen_t addr_size);

  void run_and_connect();

 protected:
  virtual void on_expander_changed();

 private:
  void ctor();
  fawkes::FawkesNetworkClient *__client;

  Gtk::Window         &__parent;
  Gtk::TreeView        __treeview;
  Gtk::Entry           __entry;
  Gtk::Expander        __expander;
  Gtk::ScrolledWindow  __scrollwin;
#ifdef HAVE_GCONFMM
  Glib::RefPtr<Gnome::Conf::Client> __gconf;
#endif

  ServiceModel *__service_model;
};

} // end of namespace fawkes

#endif
