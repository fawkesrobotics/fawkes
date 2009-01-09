
/***************************************************************************
 *  service_selector_cbe.h - Manages list of discovered services of given type
 *
 *  Created: Mon Sep 29 17:34:58 2008
 *  Copyright  2008  Daniel Beck
 *             2008  Tim Niemueller [www.niemueller.de]
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
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FawkesNetworkClient;
class ServiceModel;
class ConnectionDispatcher;

class ServiceSelectorCBE
{
 public:
  ServiceSelectorCBE( Gtk::ComboBoxEntry* services,
		      Gtk::Button* connect,
		      Gtk::Window* parent,
		      const char* service = "_fawkes._tcp" );
  ServiceSelectorCBE( Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
		      const char* cbe_name = "cbeServices",
		      const char* btn_name = "btnConnect",
		      const char* wnd_name = "wndMain",
		      const char* service = "_fawkes._tcp" );
  virtual ~ServiceSelectorCBE();

  FawkesNetworkClient* get_network_client();
  Glib::ustring get_hostname();
  unsigned int get_port();

  sigc::signal<void> signal_connected();
  sigc::signal<void> signal_disconnected();

 protected:
  void initialize();
  void on_btn_connect_clicked();
  void on_service_selected();
  void on_connected();
  void on_disconnected();

 protected:
  Gtk::ComboBoxEntry   *m_cbe_services;
  Gtk::Button          *m_btn_connect;
  Gtk::Window          *m_parent;

  ConnectionDispatcher *m_dispatcher;
  ServiceModel         *m_service_model;

 private:
   Glib::ustring  __hostname;
   unsigned short __port;
};

}
#endif /* __LIBS_GUI_UTILS_SERVICE_SELECTOR_CBE_H_ */
