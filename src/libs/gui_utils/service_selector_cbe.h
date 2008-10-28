
/***************************************************************************
 *  service_selector_cbe.h - Manages list of discovered services of given type
 *
 *  Created: Mon Sep 29 17:34:58 2008
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
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

#ifndef __LIBS_GUI_UTILS_SERVICE_SELECTOR_CBE_H_
#define __LIBS_GUI_UTILS_SERVICE_SELECTOR_CBE_H_

#include <netcomm/fawkes/client_handler.h>

#include <gtkmm.h>
#include <libglademm/xml.h>

namespace fawkes {
class FawkesNetworkClient;
class ServiceModel;

class ServiceSelectorCBE : public FawkesNetworkClientHandler
{
 public:
  ServiceSelectorCBE( Gtk::ComboBoxEntry* services,
		      Gtk::Button* connect,
		      const char* service = "_fawkes._tcp" );
  ServiceSelectorCBE( Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
		      const char* cbe_name = "cbeServices",
		      const char* btn_name = "btnConnect",
		      const char* service = "_fawkes._tcp" );
  virtual ~ServiceSelectorCBE();

  FawkesNetworkClient* get_network_client();

  sigc::signal<void> signal_connected();
  sigc::signal<void> signal_disconnected();

  // fawkes client handler
  void deregistered (unsigned int id) throw ();
  void inbound_received (FawkesNetworkMessage *m, unsigned int id) throw ();
  void connection_died (unsigned int id) throw ();
  void connection_established (unsigned int id) throw ();

 protected:
  void initialize();
  void on_btn_connect_clicked();
  void on_service_selected();
  void on_connection_established();
  void on_connection_died();

  Gtk::ComboBoxEntry* m_cbe_services;
  Gtk::Button* m_btn_connect;

  sigc::signal<void> m_signal_connected;
  sigc::signal<void> m_signal_disconnected;
  Glib::Dispatcher m_signal_connected_internal;
  Glib::Dispatcher m_signal_disconnected_internal;

  FawkesNetworkClient* m_client;
  ServiceModel* m_service_model;
};

}
#endif /* __LIBS_GUI_UTILS_SERVICE_SELECTOR_CBE_H_ */
