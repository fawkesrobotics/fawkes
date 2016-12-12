
/***************************************************************************
 *  netloggui.cpp - NetLog GUI
 *
 *  Created: Wed Nov 05 11:03:56 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "netloggui.h"

#include <gui_utils/logview.h>
#include <gui_utils/avahi_dispatcher.h>
#include <gui_utils/connection_dispatcher.h>
#include <gui_utils/service_chooser_dialog.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/dns-sd/avahi_thread.h>

#include <netinet/in.h>

using namespace fawkes;


/** @class NetLogGuiGtkWindow "netloggui.h"
 * NetLog GUI main window.
 * The NetLog GUI provides shows log viewers for Fawkes instances on the
 * network.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cobject C base object
 * @param builder Gtk builder
 */
NetLogGuiGtkWindow::NetLogGuiGtkWindow(BaseObjectType* cobject,
				       const Glib::RefPtr<Gtk::Builder> &builder)
  : Gtk::Window(cobject)
{
  builder->get_widget("vbox_main", vbox_main);
  builder->get_widget("lab_no_connection", lab_no_connection);
  builder->get_widget("tb_connection", tb_connection);
  builder->get_widget("tb_exit", tb_exit);
  builder->get_widget("tb_clear", tb_clear);

  vbox_main->pack_end(ntb_logviewers);

  avahi_dispatcher = new AvahiDispatcher();
  avahi_dispatcher->signal_service_added().connect(sigc::retype_return<void>(sigc::mem_fun(*this, &NetLogGuiGtkWindow::on_service_added)));
  avahi_dispatcher->signal_service_removed().connect(sigc::mem_fun(*this, &NetLogGuiGtkWindow::on_service_removed));

  avahi_thread = new AvahiThread();
  avahi_thread->start();
  avahi_thread->watch_service("_fawkes._tcp", avahi_dispatcher);

  tb_connection->signal_clicked().connect(sigc::mem_fun(*this, &NetLogGuiGtkWindow::on_connection_clicked));
  tb_exit->signal_clicked().connect(sigc::mem_fun(*this, &NetLogGuiGtkWindow::on_exit_clicked));
  tb_clear->signal_clicked().connect(sigc::mem_fun(*this, &NetLogGuiGtkWindow::on_clear_clicked));
}


/** Destructor. */
NetLogGuiGtkWindow::~NetLogGuiGtkWindow()
{
  avahi_thread->cancel();
  avahi_thread->join();
  delete avahi_dispatcher;
  delete avahi_thread;
}


/** Event handler for connection button. */
void
NetLogGuiGtkWindow::on_connection_clicked()
{
  ServiceChooserDialog ssd(*this);
  if (ssd.run() ) {
    struct sockaddr_in saddr;
    socklen_t saddr_size = sizeof(struct sockaddr_in);
    Glib::ustring name, hostname;
    unsigned short int port = 1910;
    std::list<std::string> txt;
    int page = -1;

    try {
      ssd.get_selected_service (name, hostname, port);
      ssd.get_raw_address((struct sockaddr *)&saddr, saddr_size);
      NetworkService *service = new NetworkService(name.c_str(), "_fawkes._tcp", "",
						   hostname.c_str(), port,
						   (struct sockaddr *)&saddr,
						   saddr_size, txt);
      page = on_service_added(service);
      delete service;

      if ( page >= 0 ) {
	Gtk::ScrolledWindow *scrolled = dynamic_cast<Gtk::ScrolledWindow *>(ntb_logviewers.get_nth_page(page));
	LogView *logview = dynamic_cast<LogView *>(scrolled->get_child());
	logview->get_client()->connect(hostname.c_str(), port);
      }
    } catch (Exception &e) {
      Glib::ustring message = *(e.begin());
      Gtk::MessageDialog md(*this, message, /* markup */ false,
			    Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			    /* modal */ true);
      md.set_title("Connection failed");
      md.run();

      ntb_logviewers.remove_page(page);
    }
  }
}


void
NetLogGuiGtkWindow::on_exit_clicked()
{
  Gtk::Main::quit();
}


void
NetLogGuiGtkWindow::on_clear_clicked()
{
  int page = ntb_logviewers.get_current_page();
  if (page >= 0) {
    Gtk::ScrolledWindow *scrolled = dynamic_cast<Gtk::ScrolledWindow *>(ntb_logviewers.get_nth_page(page));
    LogView *lv = dynamic_cast<LogView *>(scrolled->get_child());
    lv->clear();
  }
}


int
NetLogGuiGtkWindow::on_service_added(fawkes::NetworkService *service)
{
  if ( ntb_logviewers.get_n_pages() == 0 ) {
    lab_no_connection->hide();
    //Gtk::Container *thiscon = this;
    //thiscon->remove(lab_no_connection);
    //add(ntb_logviewers);
    ntb_logviewers.show();
  }

  Gtk::HBox *hbox = Gtk::manage(new Gtk::HBox(false, 4));
  Gtk::Button *button = Gtk::manage(new Gtk::Button());
  Gtk::Image *image = Gtk::manage(new Gtk::Image(Gtk::Stock::CONNECT, Gtk::ICON_SIZE_BUTTON));
  button->add(*image);
  button->set_relief(Gtk::RELIEF_NONE);
  Gtk::Label *label = Gtk::manage(new Gtk::Label());
  label->set_markup(Glib::ustring("<b>") + service->host() + "</b>\n" + service->addr_string());
  label->set_line_wrap();
  Gtk::Label *invisible = Gtk::manage(new Gtk::Label(Glib::ustring(service->name()) + "::" + service->type() + "::" + service->domain()));
  Gtk::ScrolledWindow *scrolled = Gtk::manage(new Gtk::ScrolledWindow());
  scrolled->set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
  LogView *logview =
    Gtk::manage(new LogView(service->addr_string().c_str(), service->port()));
  //scrolled->add(*logview);

  hbox->pack_start(*button);
  hbox->pack_start(*label);
  hbox->pack_start(*invisible);

  button->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &NetLogGuiGtkWindow::on_connbut_clicked), image, logview));
  logview->get_connection_dispatcher()->signal_connected().connect(sigc::bind(sigc::mem_fun(*this, &NetLogGuiGtkWindow::on_connected), image));
  logview->get_connection_dispatcher()->signal_disconnected().connect(sigc::bind(sigc::mem_fun(*this, &NetLogGuiGtkWindow::on_disconnected), image));

  scrolled->show();
  label->show();
  image->show();
  button->show();
  logview->show();
  hbox->show();

  int rv = ntb_logviewers.append_page(*logview, *hbox);

  return rv;
}


void
NetLogGuiGtkWindow::on_service_removed(fawkes::NetworkService *service)
{
  bool removed = false;
  do {
    removed = false;

    for (int i = 0; ! removed && (i < ntb_logviewers.get_n_pages()); ++i) {
      Gtk::Widget *child = ntb_logviewers.get_nth_page(i);
      Gtk::Widget *tab_label = ntb_logviewers.get_tab_label(*child);
      Gtk::HBox   *hbox = dynamic_cast<Gtk::HBox *>(tab_label);

      if ( hbox ) {
        std::vector<Gtk::Widget *> children = hbox->get_children();
	Gtk::Widget *w = children[2];
	if (w) {
	  Gtk::Label *label = dynamic_cast<Gtk::Label *>(w);
	  if ( label ) {
	    Glib::ustring s = Glib::ustring(service->name()) + "::" + service->type() + "::" + service->domain();
	    if (label->get_text() == s) {
	      ntb_logviewers.remove_page(i);
	      removed = true;
	    }
	  }
	}
      }
    }
  } while (removed);

  if ( ntb_logviewers.get_n_pages() == 0 ) {
    ntb_logviewers.hide();
    //Gtk::Container *thiscon = this;
    //thiscon->remove(ntb_logviewers);
    //add(lab_no_connection);
    lab_no_connection->show();
  }
}


void
NetLogGuiGtkWindow::on_connbut_clicked(Gtk::Image *image, fawkes::LogView *logview)
{
  FawkesNetworkClient *client = logview->get_client();
  if ( client->connected() ) {
    client->disconnect();
  } else {
    try {
      client->connect();
    } catch (Exception &e) {
      Glib::ustring message = *(e.begin());
      Gtk::MessageDialog md(*this, message, /* markup */ false,
			    Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			    /* modal */ true);
      md.set_title("Connection failed");
      md.run();
    }
  }
}


void
NetLogGuiGtkWindow::on_connected(Gtk::Image *image)
{
  image->set(Gtk::Stock::DISCONNECT, Gtk::ICON_SIZE_BUTTON);
}


void
NetLogGuiGtkWindow::on_disconnected(Gtk::Image *image)
{
  image->set(Gtk::Stock::CONNECT, Gtk::ICON_SIZE_BUTTON);
}
