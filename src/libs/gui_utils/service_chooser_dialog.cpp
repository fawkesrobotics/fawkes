
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

#include <arpa/inet.h>
#include <core/exception.h>
#include <core/exceptions/software.h>
#include <gui_utils/service_chooser_dialog.h>
#include <gui_utils/service_model.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/utils/resolver.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <utils/system/argparser.h>

#include <algorithm>
#include <cstring>

#ifdef HAVE_GCONFMM
#	define GCONF_DIR "/apps/fawkes/service_chooser_dialog"
#	define GCONF_PREFIX GCONF_DIR "/"
#endif

namespace fawkes {

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
ServiceChooserDialog::ServiceChooserDialog(Gtk::Window  &parent,
                                           Glib::ustring title,
                                           const char   *service)
: Gtk::Dialog(title, parent, /* modal */ true), parent_(parent), expander_("Manual entry")
{
	service_model_ = new ServiceModel(service);
	ctor();
	client_ = NULL;
}

/** Constructor.
 * @param parent parent window
 * @param client Fawkes network client to connect on run()
 * @param title title of the dialog
 * @param service service string
 */
ServiceChooserDialog::ServiceChooserDialog(Gtk::Window         &parent,
                                           FawkesNetworkClient *client,
                                           Glib::ustring        title,
                                           const char          *service)
: Gtk::Dialog(title, parent, /* modal */ true), parent_(parent), expander_("Manual entry")
{
	service_model_ = new ServiceModel(service);
	ctor();
	client_ = client;
}

/** Destructor. */
ServiceChooserDialog::~ServiceChooserDialog()
{
#ifdef HAVE_GCONFMM
	if (expander_.get_expanded() && !treeview_.has_focus() && entry_.get_text_length() > 0) {
		gconf_->set(GCONF_PREFIX "manual_host", entry_.get_text());
		gconf_->set(GCONF_PREFIX "manual_expanded", true);
	} else {
		gconf_->set(GCONF_PREFIX "manual_expanded", false);
	}
#endif
	delete service_model_;
}

void
ServiceChooserDialog::ctor()
{
	set_default_size(480, 300);

	treeview_.set_model(service_model_->get_list_store());
	treeview_.append_column("Service", service_model_->get_column_record().name);
	treeview_.append_column("Address/Port", service_model_->get_column_record().addrport);
	scrollwin_.add(treeview_);
	scrollwin_.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
	treeview_.show();
	expander_.add(entry_);
	entry_.show();
	entry_.set_activates_default(true);

	Glib::ustring default_host("localhost");
#ifdef HAVE_GCONFMM
	gconf_ = Gnome::Conf::Client::get_default_client();
	gconf_->add_dir(GCONF_DIR);
	Gnome::Conf::Value host_val = gconf_->get_without_default(GCONF_PREFIX "manual_host");
	if (host_val.get_type() == Gnome::Conf::VALUE_STRING) {
		default_host = host_val.get_string();
	}
#endif

	char *fawkes_ip = getenv("FAWKES_IP");
	if (fawkes_ip)
		entry_.set_text(fawkes_ip);
	else
		entry_.set_text(default_host);

	Gtk::Box *vbox = get_vbox();
	vbox->pack_start(scrollwin_);
	vbox->pack_end(expander_, Gtk::PACK_SHRINK);
	scrollwin_.show();
	expander_.show();

	add_button(Gtk::Stock::CANCEL, 0);
	add_button(Gtk::Stock::OK, 1);

	set_default_response(1);

	treeview_.signal_row_activated().connect(
	  sigc::bind(sigc::hide<0>(sigc::hide<0>(sigc::mem_fun(*this, &ServiceChooserDialog::response))),
	             1));

#ifdef GLIBMM_PROPERTIES_ENABLED
	expander_.property_expanded().signal_changed().connect(
	  sigc::mem_fun(*this, &ServiceChooserDialog::on_expander_changed));
#endif

#ifdef HAVE_GCONFMM
	if (gconf_->get_bool(GCONF_PREFIX "manual_expanded")) {
		expander_.set_expanded(true);
	}
#endif
}

/** Get selected service.
 * If a service has been selected use this method to get the IP Address as
 * string of the host that has the service and the port.
 * @param name name of the service
 * @param hostname hostname of the host associated with the service
 * @param port upon successful return contains the port
 * @exception Exception thrown if no service has been selected
 */
void
ServiceChooserDialog::get_selected_service(Glib::ustring      &name,
                                           Glib::ustring      &hostname,
                                           unsigned short int &port)
{
	Glib::RefPtr<Gtk::TreeSelection> treesel = treeview_.get_selection();
	if (expander_.get_expanded() && !treeview_.has_focus()) {
		if (entry_.get_text_length() > 0) {
			std::string tmp_hostname;
			ArgumentParser::parse_hostport_s(entry_.get_text().c_str(), tmp_hostname, port);
			hostname = tmp_hostname;
			name     = hostname;
			return;
		}
	}

	Gtk::TreeModel::iterator iter = treesel->get_selected();
	if (iter) {
		Gtk::TreeModel::Row row = *iter;
		name                    = row[service_model_->get_column_record().name];
		hostname                = row[service_model_->get_column_record().ipaddr];
		port                    = row[service_model_->get_column_record().port];

	} else {
		throw Exception("No host selected");
	}
}

/** Get selected service.
 * If a service has been selected use this method to get the IP Address as
 * string of the host that has the service and the port.
 * May not be called for manual entry since this would require hostname resolution within
 * the dialog. That should be handled on the caller's side.
 * @param hostname hostname of the host associated with the service
 * @param sockaddr upon successful return contains the sockaddr structure of the specific endpoint
 * @exception Exception thrown if no service has been selected
 */
void
ServiceChooserDialog::get_selected_service(Glib::ustring           &hostname,
                                           struct sockaddr_storage &sockaddr)
{
	Glib::RefPtr<Gtk::TreeSelection> treesel = treeview_.get_selection();
	if (expander_.get_expanded() && !treeview_.has_focus() && entry_.get_text_length() > 0) {
		throw Exception("May not be called for manual entry");
	}

	Gtk::TreeModel::iterator iter = treesel->get_selected();
	if (iter) {
		Gtk::TreeModel::Row row = *iter;
		hostname                = row[service_model_->get_column_record().ipaddr];
		sockaddr                = row[service_model_->get_column_record().sockaddr];

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
	/*
  if ( addr_size < sizeof(struct sockaddr_in) ) {
    throw Exception("Size of addrlen too small, only %u bytes, but required %zu\n",
		    addr_size, sizeof(struct sockaddr_in));
  }
  Glib::ustring name, hostname;
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
	*/
}

/** Signal handler for expander event.
 * Called when expander is (de-)expanded. Only works with Glibmm properties
 * enabled, i.e. not on Maemo.
 */
void
ServiceChooserDialog::on_expander_changed()
{
	if (expander_.get_expanded()) {
		entry_.grab_focus();
	} else {
		treeview_.grab_focus();
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
	if (!client_)
		throw NullPointerException("FawkesNetworkClient not set");
	if (client_->connected())
		throw Exception("Client is already connected");

	if (run()) {
		try {
			if (expander_.get_expanded() && !treeview_.has_focus() && entry_.get_text_length() > 0) {
				Glib::ustring      name, hostname;
				unsigned short int port;
				get_selected_service(name, hostname, port);
				client_->connect(hostname.c_str(), port);

			} else {
				struct sockaddr_storage sockaddr;
				Glib::ustring           hostname;
				get_selected_service(hostname, sockaddr);
				client_->connect(hostname.c_str(), sockaddr);
			}
		} catch (Exception &e) {
			Glib::ustring      message = *(e.begin());
			Gtk::MessageDialog md(parent_,
			                      message,
			                      /* markup */ false,
			                      Gtk::MESSAGE_ERROR,
			                      Gtk::BUTTONS_OK,
			                      /* modal */ true);
			md.set_title("Connection failed");
			md.run();
		}
	}
}

} // end of namespace fawkes
