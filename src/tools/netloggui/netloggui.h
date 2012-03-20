
/***************************************************************************
 *  netloggui.h - NetLog GUI
 *
 *  Created: Tue Nov 03 23:38:11 2008
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

#ifndef __TOOLS_NETLOGGUI_NETLOGGUI_H_
#define __TOOLS_NETLOGGUI_NETLOGGUI_H_

#include <gtkmm.h>

namespace fawkes {
  class AvahiThread;
  class LogView;
  class NetworkService;
  class AvahiDispatcher;
}

class NetLogGuiGtkWindow : public Gtk::Window
{
 public:  
  NetLogGuiGtkWindow(BaseObjectType* cobject,
		     const Glib::RefPtr<Gtk::Builder> &builder);
  ~NetLogGuiGtkWindow();

 private:
  int  on_service_added(fawkes::NetworkService *service);
  void on_service_removed(fawkes::NetworkService *service);

  void on_connection_clicked();
  void on_exit_clicked();
  void on_clear_clicked();

  void on_connbut_clicked(Gtk::Image *image, fawkes::LogView *logview);
  void on_connected(Gtk::Image *image);
  void on_disconnected(Gtk::Image *image);

 private:
  fawkes::AvahiThread     *avahi_thread;
  fawkes::AvahiDispatcher *avahi_dispatcher;

  Gtk::VBox        *vbox_main;
  Gtk::Label       *lab_no_connection;
  Gtk::ToolButton  *tb_connection;
  Gtk::ToolButton  *tb_exit;
  Gtk::ToolButton  *tb_clear;

  Gtk::Notebook ntb_logviewers; 
};

#endif
