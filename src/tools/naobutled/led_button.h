
/***************************************************************************
 *  led_button.h - Nao LED Button
 *
 *  Created: Thu Oct 30 22:05:54 2008
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

#ifndef __TOOLS_NAOBUTLED_LED_BUTTON_H_
#define __TOOLS_NAOBUTLED_LED_BUTTON_H_

#include <gtkmm.h>
#include <libglademm/xml.h>

namespace fawkes {
  class ConnectionDispatcher;
  class LedInterface;
  class Interface;
  class Message;
}

class NaoLedDrawButton : public Gtk::DrawingArea
{
 public:  
  NaoLedDrawButton(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml> &refxml);

  void set_label(Glib::ustring label);
  void set_ifaces(fawkes::LedInterface *red,
		  fawkes::LedInterface *green,
		  fawkes::LedInterface *blue);

  void set_intensities_dialog();

  void on_message_received(fawkes::Interface *interface, fawkes::Message *message);

 private:
  bool on_expose_event(GdkEventExpose* event);

 private:
  Glib::ustring __label;
  fawkes::LedInterface *__led_red_if;
  fawkes::LedInterface *__led_green_if;
  fawkes::LedInterface *__led_blue_if;

  Gtk::Dialog *dlg_intensities;
  Gtk::VScale *vsc_r;
  Gtk::VScale *vsc_g;
  Gtk::VScale *vsc_b;
};

#endif
