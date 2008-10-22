
/***************************************************************************
 *  service_selector_cbe.cpp - Manages list of discovered services of given type
 *
 *  Created: Mon Sep 29 17:46:44 2008
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

#include <gui_utils/service_selector_cbe.h>
#include <gui_utils/utils.h>
#include <netcomm/fawkes/client.h>

using namespace fawkes;

/** @class fawkes::ServiceSelectorCBE gui_utils/service_selector_cbe.h
 * This widget consists of a Gtk::ComboBoxEntry and a Gtk::Button. The
 * combo box contains all detected services of a given type; upon
 * click the button opens a network connection to the selected service.
 *
 * @author Daniel Beck
 */

/** @var fawkes::ServiceSelectorCBE::m_cbe_services
 * A Gtk::ComboBoxEntry that lists all available services.
 */

/** @var fawkes::ServiceSelectorCBE::m_btn_connect
 * A Gtk::Button that trigger the connection.
 */

/** @var fawkes::ServiceSelectorCBE::m_signal_connected
 * This signal is emitted whenever a connection is established.
 */

/** @var fawkes::ServiceSelectorCBE::m_signal_disconnected
 * This signal is emitted whenever a connection is terminated.
 */

/** @var fawkes::ServiceSelectorCBE::m_signal_connected_internal
 * This signal is emitted whenever a connection is established. It is
 * meant for class-internal signalling only!
 */

/** @var fawkes::ServiceSelectorCBE::m_signal_disconnected_internal
 * This signal is emitted whenever a connection is terminated. It is
 * meant for class-internal signalling only!
 */

/** @var fawkes::ServiceSelectorCBE::m_client
 * A network client of the selected service.
 */

/** Construtor.
 * @param services the combo box to hold the list of services
 * @param connect the button to trigger the network connection
 * @param service a service identifier
 */
ServiceSelectorCBE::ServiceSelectorCBE( Gtk::ComboBoxEntry* services,
					Gtk::Button* connect,
					const char* service )
  : ServiceView(service)
{
  m_cbe_services = services;
  m_btn_connect  = connect;

  initialize();
}

/** Constructor.
 * @param ref_xml Glade XML file
 * @param cbe_name name of the combo box
 * @param btn_name name of the button
 * @param service service identifier
 */
ServiceSelectorCBE::ServiceSelectorCBE( Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
					const char* cbe_name,
					const char* btn_name,
					const char* service )
  : ServiceView(service)
{
  m_cbe_services = dynamic_cast<Gtk::ComboBoxEntry*>( get_widget(ref_xml, cbe_name) );
  m_btn_connect  = dynamic_cast<Gtk::Button*>( get_widget(ref_xml, btn_name) );

  initialize();
}

void
ServiceSelectorCBE::initialize()
{
  m_signal_connected_internal.connect( sigc::mem_fun(*this, &ServiceSelectorCBE::on_connection_established) );
  m_signal_disconnected_internal.connect( sigc::mem_fun(*this, &ServiceSelectorCBE::on_connection_died) );

  m_cbe_services->set_model(m_service_list);
  m_cbe_services->set_text_column(m_service_record.hostname);
  m_cbe_services->get_entry()->set_activates_default(true);
  m_cbe_services->signal_changed().connect( sigc::mem_fun( *this, &ServiceSelectorCBE::on_btn_connect_clicked) );

  m_btn_connect->signal_clicked().connect( sigc::mem_fun( *this, &ServiceSelectorCBE::on_btn_connect_clicked) );
  m_btn_connect->set_label("gtk-connect");
  m_btn_connect->set_use_stock(true);
  m_btn_connect->grab_default();

  m_client = new FawkesNetworkClient();
  m_client->register_handler(this, FAWKES_CID_OBSERVER_MODE);
}

/** Destructor. */
ServiceSelectorCBE::~ServiceSelectorCBE()
{
  delete m_client;
}

/** Access the current network client.
 * @return the current network client
 */
FawkesNetworkClient*
ServiceSelectorCBE::get_network_client()
{
  return m_client;
}

/** This signal is emitted whenever a network connection is established.
 * @return reference to the corresponding dispatcher
 */
sigc::signal<void>
ServiceSelectorCBE::signal_connected()
{
  return m_signal_connected;
}

/** This signal is emitted whenever a network connection is terminated.
 * @return reference to the corresponding dispatcher
 */
sigc::signal<void>
ServiceSelectorCBE::signal_disconnected()
{
  return m_signal_disconnected;
}

/** Signal handler that is called whenever the connect button is
 * clicked or an entry in the combo box is selected.
 */
void
ServiceSelectorCBE::on_btn_connect_clicked()
{
  if ( m_client && m_client->connected() )
  { 
    m_client->disconnect();
  }
  else
  { 
    Gtk::TreeModel::Row row =	*m_cbe_services->get_active();

    Glib::ustring hostname = row[m_service_record.hostname];
    unsigned short port    = row[m_service_record.port];

    try
    {
      m_client->connect(hostname.c_str(), port);
    }
    catch (Exception& e)
    {
      e.print_trace();
    }
  }
}

/** Signal handler for the internal connection established signal. */
void
ServiceSelectorCBE::on_connection_established()
{
  m_btn_connect->set_label("gtk-disconnect");
  m_signal_connected.emit();
}

/** Signal handler for the internal connection terminated signal. */
void
ServiceSelectorCBE::on_connection_died()
{
  m_btn_connect->set_label("gtk-connect");
  m_signal_disconnected.emit();
}

void
ServiceSelectorCBE::deregistered (unsigned int id) throw ()
{
}

void
ServiceSelectorCBE::inbound_received (FawkesNetworkMessage *m, unsigned int id) throw ()
{
}

void
ServiceSelectorCBE::connection_died (unsigned int id) throw ()
{
  m_signal_disconnected_internal();
}

void
ServiceSelectorCBE::connection_established (unsigned int id) throw ()
{
  m_signal_connected_internal();
}

