
/***************************************************************************
 *  led_button.cpp - Nao LED Button
 *
 *  Created: Thu Oct 30 22:07:34 2008
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

#include "led_button.h"

#include <utils/math/angle.h>
#include <utils/math/coord.h>
#include <gui_utils/connection_dispatcher.h>
#include <interfaces/led.h>

#include <cmath>
#include <algorithm>

using namespace fawkes;


/** @class NaoLedDrawButton "led_button.h"
 * Nao LED Button Drawing Area.
 * This is a small widget derived from the Drawing Area that is used to draw
 * the status of a LED in a button container.
 * @author Tim Niemueller
 */

/** Constructor.
 * Special ctor to be used with Glade's get_widget_derived().
 * @param cobject Gtk C object
 * @param refxml Glade's XML reference
 */
NaoLedDrawButton::NaoLedDrawButton(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml> &refxml)
  : Gtk::DrawingArea(cobject)
{
  __label = "";
  __led_red_if   = NULL;
  __led_green_if = NULL;
  __led_blue_if  = NULL;

#ifndef GLIBMM_DEFAULT_SIGNAL_HANDLERS_ENABLED
  signal_expose_event().connect(sigc::mem_fun(*this, &NaoLedDrawButton::on_expose_event));
#endif
}


/** Set the label of this area.
 * @param label string to show below the LED
 */
void
NaoLedDrawButton::set_label(Glib::ustring label)
{
  __label = label;
}


/** Set LED interfaces.
 * We get one interface per color to mix the final color.
 * @param red red LED interface
 * @param green green LED interface
 * @param blue blue LED interface
 */
void
NaoLedDrawButton::set_ifaces(LedInterface *red,
			     LedInterface *green,
			     LedInterface *blue)
{
  __led_red_if   = red;
  __led_green_if = green;
  __led_blue_if  = blue;

  if (__led_red_if && __led_red_if->is_writer()) {
    __led_red_if->set_intensity(0.);
    __led_red_if->write();
  }
  if (__led_green_if && __led_green_if->is_writer()) {
    __led_green_if->set_intensity(0.);
    __led_green_if->write();
  }
  if (__led_blue_if && __led_blue_if->is_writer()) {
    __led_blue_if->set_intensity(0.);
    __led_blue_if->write();
  }

  queue_draw();
}


/** Show an intensities dialog to let the user choose the LEDs intensities. */
void
NaoLedDrawButton::set_intensities_dialog()
{
  if ( __led_red_if && __led_green_if && __led_blue_if ) {
    Gtk::ColorSelectionDialog csd("Select LED color");
    Gdk::Color color;
    color.set_rgb_p(__led_red_if->intensity(),
		    __led_green_if->intensity(),
		    __led_blue_if->intensity());
    csd.get_colorsel()->set_current_color(color);
    if (csd.run() == Gtk::RESPONSE_OK) {
      color = csd.get_colorsel()->get_current_color();
      try {
	__led_red_if->set_intensity(color.get_red_p());
	__led_green_if->set_intensity(color.get_green_p());
	__led_blue_if->set_intensity(color.get_blue_p());
	__led_red_if->write();
	__led_green_if->write();
	__led_blue_if->write();
      } catch (Exception &e) {
	Glib::ustring message = *(e.begin());
	Gtk::MessageDialog md(message, /* markup */ false,
			      Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			      /* modal */ true);
	md.set_title("Failed to set intensities");
	md.run();
      }
    }
    queue_draw();
  }
}


/** Event handler when BB message received.
 * Checks if the interface is an LedInterface and matches one of the set
 * interfaces. If this is the case the message is processed.
 * @param interface interface via which the message was received
 * @param message received message
 */
void
NaoLedDrawButton::on_message_received(fawkes::Interface *interface, Message *message)
{
  LedInterface *led_if = dynamic_cast<LedInterface *>(interface);

  if ( (led_if == NULL) ||
       ((interface != __led_red_if) &&
	(interface != __led_green_if) &&
	(interface != __led_blue_if)) ) {
    return;
  }

  LedInterface::TurnOnMessage *on_msg;
  LedInterface::TurnOffMessage *off_msg;
  LedInterface::SetIntensityMessage *int_msg;

  if ( (on_msg = dynamic_cast<LedInterface::TurnOnMessage *>(message)) != NULL ) {
    led_if->set_intensity(1.0);
  } else if ( (off_msg = dynamic_cast<LedInterface::TurnOffMessage *>(message)) != NULL ) {
    led_if->set_intensity(0.0);
  } else if ( (int_msg = dynamic_cast<LedInterface::SetIntensityMessage *>(message)) != NULL ) {
    led_if->set_intensity(int_msg->intensity());
  }

  queue_draw();
}


/** Expose event handler.
 * @param event event info structure.
 * @return signal return value
 */
bool
NaoLedDrawButton::on_expose_event(GdkEventExpose* event)
{
  // This is where we draw on the window
  Glib::RefPtr<Gdk::Window> window = get_window();
  if (window) {
    Gtk::Allocation allocation = get_allocation();
    const int width = allocation.get_width();
    const int height = allocation.get_height();
    
    // coordinates for the center of the window
    int xc, yc;
    xc = width / 2;
    yc = height / 2;
    
    Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();
    cr->set_line_width(1.0);

    // clip to the area indicated by the expose event so that we only redraw
    // the portion of the window that needs to be redrawn
    /*
    cr->set_source_rgb(1, 1, 1);
    cr->fill_preserve();
    */
    cr->rectangle(event->area.x, event->area.y,
		  event->area.width, event->area.height);
    cr->clip();

    Cairo::TextExtents te;
    //cr->set_font_size(20);
    cr->get_text_extents(__label, te);

    cr->translate(0, -te.height);

    float radius = std::min(width, height - (int)te.height * 2) / 2 - 4;

    if ( __led_red_if && __led_green_if && __led_blue_if ) {

      // this is safe even if we are a writer
      __led_red_if->read();
      __led_green_if->read();
      __led_blue_if->read();

      cr->set_source_rgb(__led_red_if->intensity(),
			 __led_green_if->intensity(),
			 __led_blue_if->intensity());
      cr->arc(xc, yc, radius, 0, 2*M_PI);
      cr->fill_preserve();
      cr->stroke();
    } else {
      cr->save();
      cr->set_source_rgb(1, 0, 0);
      cr->set_line_width(4);
      cr->arc(xc, yc, radius, 0, 2*M_PI);
      float line_x = 0, line_y = 0;
      polar2cart2d(deg2rad(45), radius, &line_x, &line_y);
      cr->move_to(xc + line_x, yc - line_y);
      polar2cart2d(deg2rad(135), radius, &line_x, &line_y);
      cr->line_to(xc + line_x, yc + line_y);
      cr->stroke();
      cr->restore();
    }

    cr->set_source_rgb(0, 0, 0);
    cr->move_to(xc - te.width / 2, height + te.height / 2);
    cr->show_text(__label);

  }

  return true;
}
