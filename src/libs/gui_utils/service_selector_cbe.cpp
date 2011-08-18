
/***************************************************************************
 *  service_selector_cbe.cpp - Manages list of discovered services of given type
 *
 *  Created: Mon Sep 29 17:46:44 2008
 *  Copyright  2008  Daniel Beck
 *             2008  Tim Niemueller [www.niemueller.de]
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
#include <gui_utils/service_model.h>
#include <gui_utils/connection_dispatcher.h>
#include <netcomm/fawkes/client.h>

#include <sstream>

using namespace fawkes;

/** @class fawkes::ServiceSelectorCBE gui_utils/service_selector_cbe.h
 * This widget consists of a Gtk::ComboBox and a Gtk::Button. The
 * combo box contains all detected services of a given type; upon
 * click the button opens a network connection to the selected service.
 *
 * @author Daniel Beck
 * @author Tim Niemueller
 */

/** @var fawkes::ServiceSelectorCBE::m_cbe_services
 * A Gtk::ComboBox that lists all available services.
 */

/** @var fawkes::ServiceSelectorCBE::m_btn_connect
 * A Gtk::Button that triggers the connection.
 */

/** @var fawkes::ServiceSelectorCBE::m_tbtn_connect
 * A Gtk::ToolButton that triggers the connection.
 */

/** @var fawkes::ServiceSelectorCBE::m_parent
 * The parent Gtk::Window.
 */

/** @var fawkes::ServiceSelectorCBE::m_service_model
 * A liststore which contains information about detected services.
 */

/** @var fawkes::ServiceSelectorCBE::m_dispatcher
 * A ConnectionDispatcher which dispatches connection signals.
 */

/** Construtor.
 * @param services the combo box to hold the list of services
 * @param connect the button to trigger the network connection
 * @param parent the parent window. Used for error dialogs.
 * @param service a service identifier
 */
#if GTK_VERSION_GE(3,0)
ServiceSelectorCBE::ServiceSelectorCBE( Gtk::ComboBox* services,
#else
ServiceSelectorCBE::ServiceSelectorCBE( Gtk::ComboBoxEntry* services,
#endif
					Gtk::Button* connect,
					Gtk::Window* parent,
					const char* service )
{
  m_service_model = new ServiceModel(service);

  m_cbe_services  = services;
  m_btn_connect   = connect;
  m_tbtn_connect  = NULL;
  m_parent        = parent;

  initialize();
}

/** Construtor.
 * @param services the combo box to hold the list of services
 * @param connect the button to trigger the network connection
 * @param parent the parent window. Used for error dialogs.
 * @param service a service identifier
 */
#if GTK_VERSION_GE(3,0)
ServiceSelectorCBE::ServiceSelectorCBE( Gtk::ComboBox* services,
#else
ServiceSelectorCBE::ServiceSelectorCBE( Gtk::ComboBoxEntry* services,
#endif
					Gtk::ToolButton* connect,
					Gtk::Window* parent,
					const char* service )
{
  m_service_model = new ServiceModel(service);

  m_cbe_services  = services;
  m_btn_connect   = NULL;
  m_tbtn_connect  = connect;
  m_parent        = parent;

  initialize();
}

/** Constructor.
 * @param builder Gtk builder
 * @param cbe_name name of the combo box
 * @param btn_name name of the button
 * @param wnd_name name of the parent window
 * @param service service identifier
 */
ServiceSelectorCBE::ServiceSelectorCBE( Glib::RefPtr<Gtk::Builder> builder,
					const char* cbe_name,
					const char* btn_name,
					const char* wnd_name,
					const char* service )
{
  m_service_model = new ServiceModel(service);

  builder->get_widget(wnd_name, m_parent);
  builder->get_widget(cbe_name, m_cbe_services);
  builder->get_widget(btn_name, m_btn_connect);

  initialize();
}

/** Initializer method. */
void
ServiceSelectorCBE::initialize()
{
#if GTK_VERSION_GE(3,0)
  if (! m_cbe_services->get_has_entry()) {
    throw Exception("Service combo box does not have an entry, fix UI file?");
  }
#endif
  m_cbe_services->set_model( m_service_model->get_list_store() );
#if GTK_VERSION_GE(3,0)
  m_cbe_services->set_entry_text_column(m_service_model->get_column_record().name);
#else
  m_cbe_services->set_text_column(m_service_model->get_column_record().name);
#endif
  m_cbe_services->get_entry()->set_activates_default(true);
  m_cbe_services->signal_changed().connect( sigc::mem_fun( *this, &ServiceSelectorCBE::on_service_selected) );
  
  Gtk::Entry *ent = static_cast<Gtk::Entry *>(m_cbe_services->get_child());
  if (ent)
  {
    char * fawkes_ip = getenv("FAWKES_IP");
    if (fawkes_ip) ent->set_text(fawkes_ip);
    else ent->set_text("localhost");
  }

  if ( m_btn_connect )
  {
    m_btn_connect->signal_clicked().connect( sigc::mem_fun( *this, &ServiceSelectorCBE::on_btn_connect_clicked) );
    m_btn_connect->set_label("gtk-connect");
    m_btn_connect->set_use_stock(true);
    m_btn_connect->grab_default();
  }
  else
  {
    m_tbtn_connect->signal_clicked().connect( sigc::mem_fun( *this, &ServiceSelectorCBE::on_btn_connect_clicked) );
    m_tbtn_connect->set_stock_id( Gtk::StockID("gtk-connect") );
    m_tbtn_connect->grab_default();
  }

  m_dispatcher = new ConnectionDispatcher();
  m_dispatcher->signal_connected().connect(sigc::mem_fun(*this, &ServiceSelectorCBE::on_connected));
  m_dispatcher->signal_disconnected().connect(sigc::mem_fun(*this, &ServiceSelectorCBE::on_disconnected));
  
  __hostname = "";
  __port = 0;
}

/** Destructor. */
ServiceSelectorCBE::~ServiceSelectorCBE()
{
  delete m_dispatcher;
  delete m_service_model;
}

/** Access the current network client.
 * @return the current network client
 */
FawkesNetworkClient*
ServiceSelectorCBE::get_network_client()
{
  return m_dispatcher->get_client();
}

/**
 * Returns the currently selected hostname (after connect)
 * @return the hostname
 */
Glib::ustring
ServiceSelectorCBE::get_hostname()
{
  return __hostname;
}

/**
 * Returns the currently selected service name (after connect)
 * @return the service name
 */
Glib::ustring
ServiceSelectorCBE::get_name()
{
  return __servicename;
}

/**
 * Returns the currently used port (after connect)
 * @return the port
 */
unsigned int
ServiceSelectorCBE::get_port()
{
  return __port;
}

/** This signal is emitted whenever a network connection is established.
 * @return reference to the corresponding dispatcher
 */
sigc::signal<void>
ServiceSelectorCBE::signal_connected()
{
  return m_dispatcher->signal_connected();
}

/** This signal is emitted whenever a network connection is terminated.
 * @return reference to the corresponding dispatcher
 */
sigc::signal<void>
ServiceSelectorCBE::signal_disconnected()
{
  return m_dispatcher->signal_disconnected();
}

/** Signal handler that is called whenever the connect button is
 * clicked or an entry in the combo box is selected.
 */
void
ServiceSelectorCBE::on_btn_connect_clicked()
{
  FawkesNetworkClient *client = m_dispatcher->get_client();

  if (client->connected())
  {
    client->disconnect();
    if ( m_btn_connect )
    { m_btn_connect->set_label("gtk-connect"); }
    else
    { m_tbtn_connect->set_label("gtk-connect"); }
  }
  else
  { 
    if ( -1 == m_cbe_services->get_active_row_number() )
    {
      Gtk::Entry* entry = m_cbe_services->get_entry();
      __hostname = entry->get_text();

      Glib::ustring::size_type pos;
      if ((pos = __hostname.find(':')) != Glib::ustring::npos) 
      {
        Glib::ustring host = "";
        unsigned int port = 1234567; //Greater than max port num (i.e. 65535)
        std::istringstream is(__hostname.replace(pos, 1, " "));
        is >> host;
        is >> port;
        
        if (port != 1234567 && host.size())
        {
          __hostname = host;
          __port = port;
        }
      }
      else __port = 1910;
      __servicename = __hostname;
    }
    else
    {
      Gtk::TreeModel::Row row = *m_cbe_services->get_active();
      __hostname = row[m_service_model->get_column_record().hostname];
      __servicename = row[m_service_model->get_column_record().name];
      __port = row[m_service_model->get_column_record().port];
    }

    try
    {
      client->connect( __hostname.c_str(), __port );
    }
    catch (Exception& e)
    {
      Glib::ustring message = *(e.begin());
      Gtk::MessageDialog md(*m_parent, message, /* markup */ false,
			    Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			    /* modal */ true);
      md.set_title("Connection failed");
      md.run();
    }
  }
}

/** Signal handler that is called whenever an entry is selected from
 * the combo box.
 */
void
ServiceSelectorCBE::on_service_selected()
{
  if ( -1 == m_cbe_services->get_active_row_number() )  return;

  FawkesNetworkClient *client = m_dispatcher->get_client();
  if ( client->connected() )
  {
    client->disconnect();
  }

  Gtk::TreeModel::Row row = *m_cbe_services->get_active();
  __hostname = row[m_service_model->get_column_record().hostname];
  __servicename = row[m_service_model->get_column_record().name];
  __port = row[m_service_model->get_column_record().port];

  m_cbe_services->get_entry()->set_text(__hostname);

  try
  {
    client->connect( __hostname.c_str(), __port );
  }
  catch (Exception& e)
  {
    Glib::ustring message = *(e.begin());
    Gtk::MessageDialog md(*m_parent, message, /* markup */ false,
			  Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			  /* modal */ true);
    md.set_title("Connection failed");
    md.run();
  }
}

/** Signal handler for the connection established signal. */
void
ServiceSelectorCBE::on_connected()
{
  if ( m_btn_connect )
  { m_btn_connect->set_label("gtk-disconnect"); }
  else
  { m_tbtn_connect->set_stock_id( Gtk::StockID("gtk-disconnect") ); }
}

/** Signal handler for the connection terminated signal. */
void
ServiceSelectorCBE::on_disconnected()
{
  if ( m_btn_connect )
  { m_btn_connect->set_label("gtk-connect"); }
  else
  { m_tbtn_connect->set_stock_id( Gtk::StockID("gtk-connect") ); }
}
