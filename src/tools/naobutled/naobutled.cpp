
/***************************************************************************
 *  naobutled.cpp - Nao Button/LED GUI
 *
 *  Created: Thu Oct 30 21:39:45 2008
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

#include "naobutled.h"

#include <utils/system/argparser.h>
#include <blackboard/remote.h>
#include <interfaces/led.h>
#include <interfaces/switch.h>
#include <netcomm/fawkes/client.h>
#include <interface/interface_info.h>

#include <gui_utils/service_chooser_dialog.h>
#include <gui_utils/interface_dispatcher.h>

#include <cstring>

#define CHESTBUT_TIMEOUT_INTERVAL 20
#define CHESTBUT_RESET_TIMEOUT_INTERVAL 2000

using namespace fawkes;


/** @class NaoButLedGtkWindow "naogui.h"
 * Nao Button/LED GUI main window.
 * The Nao GUI provides access to the buttons and LEDs of the robot (to provide
 * access to this during the simulation.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cobject C base object
 * @param refxml Glade XML
 */
NaoButLedGtkWindow::NaoButLedGtkWindow(BaseObjectType* cobject,
				 const Glib::RefPtr<Gnome::Glade::Xml> &refxml)
  : Gtk::Window(cobject)
{
  bb = NULL;
  __chestbut_if = NULL;
  __led_chest_red_if = __led_chest_green_if = __led_chest_blue_if = NULL;
  __led_leftfoot_red_if = __led_leftfoot_green_if = __led_leftfoot_blue_if = NULL;
  __led_rightfoot_red_if = __led_rightfoot_green_if = __led_rightfoot_blue_if = NULL;
  __ifd_chestbut = NULL;
  __ifd_chestled_red = __ifd_chestled_green = __ifd_chestled_blue = NULL;
  __ifd_leftfoot_red = __ifd_leftfoot_green = __ifd_leftfoot_blue = NULL;
  __ifd_rightfoot_red = __ifd_rightfoot_green = __ifd_rightfoot_blue = NULL;

  refxml->get_widget("lab_enabled", lab_enabled);
  refxml->get_widget("lab_history", lab_history);
  refxml->get_widget("lab_activations", lab_activations);
  refxml->get_widget("lab_short", lab_short);
  refxml->get_widget("lab_long", lab_long);

  refxml->get_widget("but_chestbutton", but_chestbutton);
  refxml->get_widget("but_connection", but_connection);
  refxml->get_widget("but_leftfoot", but_leftfoot);
  refxml->get_widget("but_chestled", but_chestled);
  refxml->get_widget("but_rightfoot", but_rightfoot);
  refxml->get_widget("img_connection", img_connection);

  refxml->get_widget_derived("draw_leftfoot", draw_leftfoot);
  refxml->get_widget_derived("draw_chestled", draw_chestled);
  refxml->get_widget_derived("draw_rightfoot", draw_rightfoot);

  draw_leftfoot->set_label("LFoot");
  draw_chestled->set_label("Chest");
  draw_rightfoot->set_label("RFoot");

  but_chestbutton->set_sensitive(false);
  but_leftfoot->set_sensitive(false);
  but_chestled->set_sensitive(false);
  but_rightfoot->set_sensitive(false);

  connection_dispatcher.signal_connected().connect(sigc::mem_fun(*this, &NaoButLedGtkWindow::on_connect));
  connection_dispatcher.signal_disconnected().connect(sigc::mem_fun(*this, &NaoButLedGtkWindow::on_disconnect));

  but_leftfoot->signal_clicked().connect(sigc::mem_fun(*draw_leftfoot, &NaoLedDrawButton::set_intensities_dialog));
  but_chestled->signal_clicked().connect(sigc::mem_fun(*draw_chestled, &NaoLedDrawButton::set_intensities_dialog));
  but_rightfoot->signal_clicked().connect(sigc::mem_fun(*draw_rightfoot, &NaoLedDrawButton::set_intensities_dialog));
  but_connection->signal_clicked().connect(sigc::mem_fun(*this, &NaoButLedGtkWindow::on_connection_clicked));

  but_chestbutton->signal_pressed().connect(sigc::mem_fun(*this, &NaoButLedGtkWindow::on_chestbut_pressed));
  but_chestbutton->signal_released().connect(sigc::mem_fun(*this, &NaoButLedGtkWindow::on_chestbut_released));
}


/** Destructor. */
NaoButLedGtkWindow::~NaoButLedGtkWindow()
{
}


/** Event handler for connection button. */
void
NaoButLedGtkWindow::on_connection_clicked()
{
  if ( ! connection_dispatcher.get_client()->connected() ) {
    ServiceChooserDialog ssd(*this, connection_dispatcher.get_client());
    ssd.run_and_connect();
  } else {
    connection_dispatcher.get_client()->disconnect();
  }  
}


void
NaoButLedGtkWindow::on_chestbut_pressed()
{
  if ( __chestbut_if->is_writer() ) {
    if ( __chestbut_reset_timeout_con.connected() ) {
      __chestbut_reset_timeout_con.disconnect();
    }
    __chestbut_if->set_enabled(true);
    __chestbut_if->set_history(0);
    __chestbut_if->write();
    __chestbut_timeout_con = Glib::signal_timeout().connect(sigc::mem_fun(*this, &NaoButLedGtkWindow::on_chestbut_timeout), CHESTBUT_TIMEOUT_INTERVAL);
  } else {
    __chestbut_if->msgq_enqueue(new SwitchInterface::EnableSwitchMessage());
  }
}


void
NaoButLedGtkWindow::on_chestbut_released()
{
  if ( __chestbut_if->is_writer() ) {
    __chestbut_timeout_con.disconnect();
  
    if ( __chestbut_if->history() > 0.05 ) {
      __chestbut_if->set_activation_count(__chestbut_if->activation_count() + 1);
      if ( __chestbut_if->history() > 0.5 ) {
	__chestbut_if->set_long_activations(__chestbut_if->long_activations() + 1);
      } else {
	__chestbut_if->set_short_activations(__chestbut_if->short_activations() + 1);
      }
    }

    __chestbut_if->set_enabled(false);
    __chestbut_if->set_history(0);
    __chestbut_if->write();

    __chestbut_reset_timeout_con = Glib::signal_timeout().connect(sigc::mem_fun(*this, &NaoButLedGtkWindow::on_chestbut_reset_timeout), CHESTBUT_RESET_TIMEOUT_INTERVAL);
  } else {
    __chestbut_if->msgq_enqueue(new SwitchInterface::DisableSwitchMessage());
  }
}


bool
NaoButLedGtkWindow::on_chestbut_timeout()
{
  __chestbut_if->set_history(__chestbut_if->history() + CHESTBUT_TIMEOUT_INTERVAL / 1000.f);
  __chestbut_if->write();

  return true;
}


bool
NaoButLedGtkWindow::on_chestbut_reset_timeout()
{
  __chestbut_if->set_short_activations(0);
  __chestbut_if->set_long_activations(0);
  __chestbut_if->set_history(-2.0);
  __chestbut_if->write();

  return false;
}


void
NaoButLedGtkWindow::on_chestbut_data_change()
{
  __chestbut_if->read();
  Glib::ustring tmp = (__chestbut_if->is_enabled() ? "true" : "false");
  lab_enabled->set_text(tmp);

#if GLIBMM_MAJOR_VERSION > 2 || ( GLIBMM_MAJOR_VERSION == 2 && GLIBMM_MINOR_VERSION >= 16 )
  tmp = Glib::ustring::compose("%1", __chestbut_if->history());
  lab_history->set_text(tmp);
  tmp = Glib::ustring::compose("%1", __chestbut_if->activation_count());
  lab_activations->set_text(tmp);
  tmp = Glib::ustring::compose("%1", __chestbut_if->long_activations());
  lab_long->set_text(tmp);
  tmp = Glib::ustring::compose("%1", __chestbut_if->short_activations());
  lab_short->set_text(tmp);
#else
  char *ctmp;
  asprintf(&ctmp, "%0.2f", __chestbut_if->history());
  lab_history->set_text(ctmp);
  free(ctmp);
  asprintf(&ctmp, "%u", __chestbut_if->activation_count());
  lab_activations->set_text(ctmp);
  free(ctmp);
  asprintf(&ctmp, "%u", __chestbut_if->long_activations());
  lab_long->set_text(ctmp);
  free(ctmp);
  asprintf(&ctmp, "%u", __chestbut_if->short_activations());
  lab_short->set_text(ctmp);
  free(ctmp);
#endif

  lab_enabled->queue_draw();
  lab_history->queue_draw();
  lab_activations->queue_draw();
  lab_short->queue_draw();
  lab_long->queue_draw();
}


void
NaoButLedGtkWindow::close_bb()
{
  if ( bb ) {
    bb->close(__chestbut_if);
    
    bb->close(__led_chest_red_if);
    bb->close(__led_chest_green_if);
    bb->close(__led_chest_blue_if);

    bb->close(__led_leftfoot_red_if);
    bb->close(__led_leftfoot_green_if);
    bb->close(__led_leftfoot_blue_if);

    bb->close(__led_rightfoot_red_if);
    bb->close(__led_rightfoot_green_if);
    bb->close(__led_rightfoot_blue_if);

    delete bb;
    delete __ifd_chestbut;

    delete __ifd_chestled_red;
    delete __ifd_leftfoot_red;
    delete __ifd_rightfoot_red;

    delete __ifd_chestled_green;
    delete __ifd_leftfoot_green;
    delete __ifd_rightfoot_green;

    delete __ifd_chestled_blue;
    delete __ifd_leftfoot_blue;
    delete __ifd_rightfoot_blue;

    __chestbut_if = NULL;
    __led_chest_red_if = __led_chest_green_if = __led_chest_blue_if = NULL;
    __led_leftfoot_red_if = __led_leftfoot_green_if = __led_leftfoot_blue_if = NULL;
    __led_rightfoot_red_if = __led_rightfoot_green_if = __led_rightfoot_blue_if = NULL;
    __ifd_chestbut = NULL;
    __ifd_chestled_red = __ifd_chestled_green = __ifd_chestled_blue = NULL;
    __ifd_leftfoot_red = __ifd_leftfoot_green = __ifd_leftfoot_blue = NULL;
    __ifd_rightfoot_red = __ifd_rightfoot_green = __ifd_rightfoot_blue = NULL;
    bb = NULL;
  }
}

/** Event handler for connected event. */
void
NaoButLedGtkWindow::on_connect()
{
  try {
    bb               = new RemoteBlackBoard(connection_dispatcher.get_client());

    bool client_mode = false;
    InterfaceInfoList *iil = bb->list_all();
    for (InterfaceInfoList::iterator i = iil->begin(); i != iil->end(); ++i) {
      if ( (strcmp(i->type(), "SwitchInterface") == 0) &&
	   (strcmp(i->id(), "Nao ChestButton") == 0) ) {
	client_mode = i->has_writer();
	break;
      }
    }
    delete iil;

    unsigned int listener_flags;

    if ( client_mode ) {
      __chestbut_if = bb->open_for_reading<SwitchInterface>("Nao ChestButton");

      __led_chest_red_if   = bb->open_for_reading<LedInterface>("Nao LED ChestBoard/Red");
      __led_chest_green_if = bb->open_for_reading<LedInterface>("Nao LED ChestBoard/Green");
      __led_chest_blue_if  = bb->open_for_reading<LedInterface>("Nao LED ChestBoard/Blue");

      __led_leftfoot_red_if   = bb->open_for_reading<LedInterface>("Nao LED LFoot/Red");
      __led_leftfoot_green_if = bb->open_for_reading<LedInterface>("Nao LED LFoot/Green");
      __led_leftfoot_blue_if  = bb->open_for_reading<LedInterface>("Nao LED LFoot/Blue");

      __led_rightfoot_red_if   = bb->open_for_reading<LedInterface>("Nao LED RFoot/Red");
      __led_rightfoot_green_if = bb->open_for_reading<LedInterface>("Nao LED RFoot/Green");
      __led_rightfoot_blue_if  = bb->open_for_reading<LedInterface>("Nao LED RFoot/Blue");

      listener_flags = BlackBoard::BBIL_FLAG_DATA;
    } else {
      __chestbut_if = bb->open_for_writing<SwitchInterface>("Nao ChestButton");

      __led_chest_red_if   = bb->open_for_writing<LedInterface>("Nao LED ChestBoard/Red");
      __led_chest_green_if = bb->open_for_writing<LedInterface>("Nao LED ChestBoard/Green");
      __led_chest_blue_if  = bb->open_for_writing<LedInterface>("Nao LED ChestBoard/Blue");

      __led_leftfoot_red_if   = bb->open_for_writing<LedInterface>("Nao LED LFoot/Red");
      __led_leftfoot_green_if = bb->open_for_writing<LedInterface>("Nao LED LFoot/Green");
      __led_leftfoot_blue_if  = bb->open_for_writing<LedInterface>("Nao LED LFoot/Blue");

      __led_rightfoot_red_if   = bb->open_for_writing<LedInterface>("Nao LED RFoot/Red");
      __led_rightfoot_green_if = bb->open_for_writing<LedInterface>("Nao LED RFoot/Green");
      __led_rightfoot_blue_if  = bb->open_for_writing<LedInterface>("Nao LED RFoot/Blue");

      listener_flags = BlackBoard::BBIL_FLAG_MESSAGES;
    }

    draw_leftfoot->set_ifaces(__led_leftfoot_red_if, __led_leftfoot_green_if, __led_leftfoot_blue_if);
    draw_chestled->set_ifaces(__led_chest_red_if, __led_chest_green_if, __led_chest_blue_if);
    draw_rightfoot->set_ifaces(__led_rightfoot_red_if, __led_rightfoot_green_if, __led_rightfoot_blue_if);

    __ifd_chestbut = new InterfaceDispatcher("ChestButton", __chestbut_if);

    __ifd_chestled_red   = new InterfaceDispatcher("ChestLed Red", __led_chest_red_if, false);
    __ifd_chestled_green = new InterfaceDispatcher("ChestLed Green", __led_chest_green_if, false);
    __ifd_chestled_blue  = new InterfaceDispatcher("ChestLed Blue", __led_chest_blue_if, false);

    __ifd_leftfoot_red   = new InterfaceDispatcher("Leftfoot Red", __led_leftfoot_red_if, false);
    __ifd_leftfoot_green = new InterfaceDispatcher("Leftfoot Green", __led_leftfoot_green_if, false);
    __ifd_leftfoot_blue  = new InterfaceDispatcher("Leftfoot Blue", __led_leftfoot_blue_if, false);

    __ifd_rightfoot_red   = new InterfaceDispatcher("Rightfoot Red", __led_rightfoot_red_if, false);
    __ifd_rightfoot_green = new InterfaceDispatcher("Rightfoot Green", __led_rightfoot_green_if, false);
    __ifd_rightfoot_blue  = new InterfaceDispatcher("Rightfoot Blue", __led_rightfoot_blue_if, false);

    __ifd_chestbut->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*this, &NaoButLedGtkWindow::on_chestbut_data_change)));

    if (client_mode) {
      __ifd_chestled_red->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*draw_chestled, &NaoLedDrawButton::queue_draw)));
      __ifd_chestled_green->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*draw_chestled, &NaoLedDrawButton::queue_draw)));
      __ifd_chestled_blue->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*draw_chestled, &NaoLedDrawButton::queue_draw)));

      __ifd_leftfoot_red->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*draw_leftfoot, &NaoLedDrawButton::queue_draw)));
      __ifd_leftfoot_green->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*draw_leftfoot, &NaoLedDrawButton::queue_draw)));
      __ifd_leftfoot_blue->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*draw_leftfoot, &NaoLedDrawButton::queue_draw)));

      __ifd_rightfoot_red->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*draw_rightfoot, &NaoLedDrawButton::queue_draw)));
      __ifd_rightfoot_green->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*draw_rightfoot, &NaoLedDrawButton::queue_draw)));
      __ifd_rightfoot_blue->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*draw_rightfoot, &NaoLedDrawButton::queue_draw)));
    } else {
      __ifd_chestled_red->signal_message_received().connect(sigc::mem_fun(*draw_chestled, &NaoLedDrawButton::on_message_received));
      __ifd_chestled_green->signal_message_received().connect(sigc::mem_fun(*draw_chestled, &NaoLedDrawButton::on_message_received));
      __ifd_chestled_blue->signal_message_received().connect(sigc::mem_fun(*draw_chestled, &NaoLedDrawButton::on_message_received));

      __ifd_leftfoot_red->signal_message_received().connect(sigc::mem_fun(*draw_leftfoot, &NaoLedDrawButton::on_message_received));
      __ifd_leftfoot_green->signal_message_received().connect(sigc::mem_fun(*draw_leftfoot, &NaoLedDrawButton::on_message_received));
      __ifd_leftfoot_blue->signal_message_received().connect(sigc::mem_fun(*draw_leftfoot, &NaoLedDrawButton::on_message_received));

      __ifd_rightfoot_red->signal_message_received().connect(sigc::mem_fun(*draw_rightfoot, &NaoLedDrawButton::on_message_received));
      __ifd_rightfoot_green->signal_message_received().connect(sigc::mem_fun(*draw_rightfoot, &NaoLedDrawButton::on_message_received));
      __ifd_rightfoot_blue->signal_message_received().connect(sigc::mem_fun(*draw_rightfoot, &NaoLedDrawButton::on_message_received));
    }

    bb->register_listener(__ifd_chestbut, BlackBoard::BBIL_FLAG_DATA);

    bb->register_listener(__ifd_chestled_red, listener_flags);
    bb->register_listener(__ifd_chestled_green, listener_flags);
    bb->register_listener(__ifd_chestled_blue, listener_flags);

    bb->register_listener(__ifd_leftfoot_red, listener_flags);
    bb->register_listener(__ifd_leftfoot_green, listener_flags);
    bb->register_listener(__ifd_leftfoot_blue, listener_flags);

    bb->register_listener(__ifd_rightfoot_red, listener_flags);
    bb->register_listener(__ifd_rightfoot_green, listener_flags);
    bb->register_listener(__ifd_rightfoot_blue, listener_flags);

    img_connection->set(Gtk::Stock::DISCONNECT, Gtk::ICON_SIZE_BUTTON);

    but_chestbutton->set_sensitive(true);
    but_leftfoot->set_sensitive(! client_mode);
    but_chestled->set_sensitive(! client_mode);
    but_rightfoot->set_sensitive(! client_mode);

  } catch (Exception &e) {
    Glib::ustring message = *(e.begin());
    Gtk::MessageDialog md(*this, message, /* markup */ false,
			  Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			  /* modal */ true);
    md.set_title("BlackBoard connection failed");
    md.run();

    close_bb();
    connection_dispatcher.get_client()->disconnect();
  }
}

/** Event handler for disconnected event. */
void
NaoButLedGtkWindow::on_disconnect()
{
  draw_leftfoot->set_ifaces(NULL, NULL, NULL);
  draw_chestled->set_ifaces(NULL, NULL, NULL);
  draw_rightfoot->set_ifaces(NULL, NULL, NULL);

  but_chestbutton->set_sensitive(false);
  but_leftfoot->set_sensitive(false);
  but_chestled->set_sensitive(false);
  but_rightfoot->set_sensitive(false);

  close_bb();

  img_connection->set(Gtk::Stock::CONNECT, Gtk::ICON_SIZE_BUTTON);
}
