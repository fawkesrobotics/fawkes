
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

#include <core/exception.h>
#include <core/exceptions/software.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/utils/resolver.h>
#include <gui_utils/service_chooser_dialog.h>
#include <gui_utils/service_model.h>

#include <algorithm>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#ifdef HAVE_GCONFMM
#  define GCONF_PREFIX "/apps/fawkes/service_chooser_dialog/"
#endif

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ServiceChooserDialog <gui_utils/service_chooser_dialog.h>
 * Service chooser dialog.
 * Allows to choose a service discovered via Avahi. Use the run routine,
 * it returns 1 if a service was selected or 0 if no service was found or
 * the selection was cancelled. The dialog is always modal.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param parent parent window
 * @param title title of the dialog
 * @param service service string
 */
ServiceChooserDialog::ServiceChooserDialog(Gtk::Window &parent,
					   Glib::ustring title,
					   const char *service)
  : Gtk::Dialog(title, parent, /* modal */ true),
    __parent(parent), __expander("Manual entry")
{
  __service_model = new ServiceModel(service);
  ctor();
  __client = NULL;
}


/** Constructor.
 * @param parent parent window
 * @param client Fawkes network client to connect on run()
 * @param title title of the dialog
 * @param service service string
 */
ServiceChooserDialog::ServiceChooserDialog(Gtk::Window &parent,
					   FawkesNetworkClient *client,
					   Glib::ustring title,
					   const char *service)
  : Gtk::Dialog(title, parent, /* modal */ true),
    __parent(parent), __expander("Manual entry")
{
  __service_model = new ServiceModel(service);
  ctor();
  __client = client;
}


/** Destructor. */
ServiceChooserDialog::~ServiceChooserDialog()
{
#ifdef HAVE_GCONFMM
  if (__expander.get_expanded() && ! __treeview.has_focus() && __entry.get_text_length() > 0 ) {
    __gconf->set(GCONF_PREFIX"manual_host", __entry.get_text());
    __gconf->set(GCONF_PREFIX"manual_expanded", true);
  } else {
    __gconf->set(GCONF_PREFIX"manual_expanded", false);
  }
#endif
  delete __service_model;
}


void
ServiceChooserDialog::ctor()
{
  set_default_size(360, 240);

  __treeview.set_model(__service_model->get_list_store());
  __treeview.append_column("Service", __service_model->get_column_record().name);
  __treeview.append_column("IP Address", __service_model->get_column_record().ipaddr);
  __scrollwin.add(__treeview);
  __scrollwin.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
  __treeview.show();
  __expander.add(__entry);
  __entry.show();
  __entry.set_activates_default(true);

  Glib::ustring default_host("localhost");
#ifdef HAVE_GCONFMM
  __gconf = Gnome::Conf::Client::get_default_client();
  __gconf->add_dir(GCONF_PREFIX);
  Gnome::Conf::Value host_val =
    __gconf->get_without_default(GCONF_PREFIX"manual_host");
  if (host_val.get_type() == Gnome::Conf::VALUE_STRING) {
    default_host = host_val.get_string();
  }
#endif

  char * fawkes_ip = getenv("FAWKES_IP");
  if (fawkes_ip) __entry.set_text(fawkes_ip);
  else __entry.set_text(default_host);

  Gtk::Box *vbox = get_vbox();
  vbox->pack_start(__scrollwin);
  vbox->pack_end(__expander, Gtk::PACK_SHRINK);
  __scrollwin.show();
  __expander.show();

  add_button(Gtk::Stock::CANCEL, 0);
  add_button(Gtk::Stock::OK, 1);

  set_default_response(1);

  __treeview.signal_row_activated().connect(sigc::bind(sigc::hide<0>(sigc::hide<0>(sigc::mem_fun(*this, &ServiceChooserDialog::response))), 1));


#ifdef GLIBMM_PROPERTIES_ENABLED
  __expander.property_expanded().signal_changed().connect(sigc::mem_fun(*this, &ServiceChooserDialog::on_expander_changed));
#endif

#ifdef HAVE_GCONFMM
  if (__gconf->get_bool(GCONF_PREFIX"manual_expanded")) {
    __expander.set_expanded(true);
  }
#endif
}


/** Get selected service.
 * If a service has been selected use this method to get the IP Address as
 * string of the host that has the service and the port.
 * @param name name of the service
 * @param hostname hostname of the host associated with the service
 * @param ipaddr upon successful return contains the IP address as string
 * @param port upon successful return contains the port
 * @exception Exception thrown if no service has been selected
 */
void
ServiceChooserDialog::get_selected_service(Glib::ustring &name,
					   Glib::ustring &hostname,
					   Glib::ustring &ipaddr,
					   unsigned short int &port)
{
  Glib::RefPtr<Gtk::TreeSelection> treesel = __treeview.get_selection();
  if (__expander.get_expanded() && !__treeview.has_focus()) {
    if ( __entry.get_text_length() > 0 ) {
      char *tmpvalue = strdup(__entry.get_text().c_str());

      if ( strchr(tmpvalue, ':') != NULL ) {
	char *save_ptr;
	hostname = strtok_r(tmpvalue, ":", &save_ptr);
	char *tmpport = strtok_r(NULL, "", &save_ptr);

	int port_num = atoi(tmpport);
	if ( (port_num < 0) || (port_num > 0xFFFF) ) {
	  throw OutOfBoundsException("Invalid port", port_num, 0, 0xFFFF);
	}
	port = port_num;
      } else {
	hostname = tmpvalue;
	port = 0;
      }

      // evil, but Glib::Regex is only availabel in ver >= 2.14, n/a on maemo
      ipaddr = hostname;

      name = hostname;

      free(tmpvalue);
      return;
    }
  }

  Gtk::TreeModel::iterator iter = treesel->get_selected();
  if (iter) {
    Gtk::TreeModel::Row row = *iter;
    name     = row[__service_model->get_column_record().name];
    hostname = row[__service_model->get_column_record().hostname];
    ipaddr   = row[__service_model->get_column_record().ipaddr];
    port     = row[__service_model->get_column_record().port];

  } else {
    throw Exception("No host selected");
  }
}


/** Get raw address.
 * @param addr upon returns contains the raw representation of the IP address
 * @param addr_size size in bytes of addr, if addr_size is too small for an
 * AF_INET addr an exception is thrown.
 */
void
ServiceChooserDialog::get_raw_address(struct sockaddr *addr, socklen_t addr_size)
{
  if ( addr_size < sizeof(struct sockaddr_in) ) {
    throw Exception("Size of addrlen too small, only %u bytes, but required %zu\n",
		    addr_size, sizeof(struct sockaddr_in));
  }
  Glib::ustring name, hostname, ipaddr;
  unsigned short int port;
  get_selected_service (name, hostname, ipaddr, port);

  if (inet_pton(AF_INET, ipaddr.c_str(), &(((struct sockaddr_in *)addr)->sin_addr)) <= 0) {
    NetworkNameResolver resolver;
    struct sockaddr_in *saddr;
    socklen_t saddr_len;
    if (resolver.resolve_name_blocking(ipaddr.c_str(), (struct sockaddr **)&saddr, &saddr_len)) {
      memcpy(addr, saddr, std::min(saddr_len, addr_size));
    } else {
      throw Exception("Could not lookup hostname '%s' and it is not a valid IP address",
		      ipaddr.c_str());
    }
  }

}


/** Signal handler for expander event.
 * Called when expander is (de-)expanded. Only works with Glibmm properties
 * enabled, i.e. not on Maemo.
 */
void
ServiceChooserDialog::on_expander_changed()
{
  if (__expander.get_expanded()) {
    __entry.grab_focus();
  } else {
    __treeview.grab_focus();
  }
}


/** Run dialog and try to connect.
 * This runs the service chooser dialog and connects to the given service
 * with the attached FawkesNetworkClient. If the connection couldn't be established
 * an error dialog is shown. You should not rely on the connection to be
 * active after calling this method, rather you should use a ConnectionDispatcher
 * to get the "connected" signal.
 */
void
ServiceChooserDialog::run_and_connect()
{
  if (! __client)  throw NullPointerException("FawkesNetworkClient not set");
  if (__client->connected()) throw Exception("Client is already connected");

  if ( run() ) {
    try {
      Glib::ustring name;
      Glib::ustring hostname;
      Glib::ustring ipaddr;
      unsigned short int port;
      get_selected_service(name, hostname, ipaddr, port);
      if ( port == 0 )  port = 1910;

      __client->connect(hostname.c_str(), ipaddr.c_str(), port);
    } catch (Exception &e) {
      Glib::ustring message = *(e.begin());
      Gtk::MessageDialog md(__parent, message, /* markup */ false,
			    Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			    /* modal */ true);
      md.set_title("Connection failed");
      md.run();
    }
  }
}

} // end of namespace fawkes
