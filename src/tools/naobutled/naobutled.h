
/***************************************************************************
 *  naobutled.cpp - Nao Button/LED GUI
 *
 *  Created: Thu Oct 30 21:12:02 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __TOOLS_NAOBUTLED_NAOBUTLED_H_
#define __TOOLS_NAOBUTLED_NAOBUTLED_H_

#include "led_button.h"

#include <gui_utils/connection_dispatcher.h>

#include <gtkmm.h>
#include <libglademm/xml.h>

namespace fawkes {
  class BlackBoard;
  class LedInterface;
  class InterfaceDispatcher;
  class SwitchInterface;
}

class NaoButLedGtkWindow : public Gtk::Window
{
 public:  
  NaoButLedGtkWindow(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml> &refxml);
  ~NaoButLedGtkWindow();

 private:
  void close_bb();

  void on_connection_clicked();
  void on_connect();
  void on_disconnect();
  void on_chestbut_pressed();
  void on_chestbut_released();
  bool on_chestbut_timeout();
  bool on_chestbut_reset_timeout();

  void on_chestbut_data_change();

 private:
  fawkes::BlackBoard *bb;

  fawkes::ConnectionDispatcher connection_dispatcher;

  fawkes::InterfaceDispatcher  *__ifd_chestbut;

  fawkes::InterfaceDispatcher  *__ifd_chestled_red;
  fawkes::InterfaceDispatcher  *__ifd_leftfoot_red;
  fawkes::InterfaceDispatcher  *__ifd_rightfoot_red;

  fawkes::InterfaceDispatcher  *__ifd_chestled_green;
  fawkes::InterfaceDispatcher  *__ifd_leftfoot_green;
  fawkes::InterfaceDispatcher  *__ifd_rightfoot_green;

  fawkes::InterfaceDispatcher  *__ifd_chestled_blue;
  fawkes::InterfaceDispatcher  *__ifd_leftfoot_blue;
  fawkes::InterfaceDispatcher  *__ifd_rightfoot_blue;

  Gtk::Label  *lab_enabled;
  Gtk::Label  *lab_history;
  Gtk::Label  *lab_activations;
  Gtk::Label  *lab_short;
  Gtk::Label  *lab_long;

  Gtk::Button *but_chestbutton;
  Gtk::Button *but_connection;
  Gtk::Button *but_leftfoot;
  Gtk::Button *but_chestled;
  Gtk::Button *but_rightfoot;

  NaoLedDrawButton *draw_leftfoot;
  NaoLedDrawButton *draw_chestled;
  NaoLedDrawButton *draw_rightfoot;

  Gtk::Image  *img_connection;

  fawkes::SwitchInterface *__chestbut_if;

  fawkes::LedInterface *__led_chest_red_if;
  fawkes::LedInterface *__led_chest_green_if;
  fawkes::LedInterface *__led_chest_blue_if;

  fawkes::LedInterface *__led_leftfoot_red_if;
  fawkes::LedInterface *__led_leftfoot_green_if;
  fawkes::LedInterface *__led_leftfoot_blue_if;

  fawkes::LedInterface *__led_rightfoot_red_if;
  fawkes::LedInterface *__led_rightfoot_green_if;
  fawkes::LedInterface *__led_rightfoot_blue_if;

  sigc::connection __chestbut_timeout_con;
  sigc::connection __chestbut_reset_timeout_con;
};

#endif
