
/***************************************************************************
 *  lasergui_hildon.cpp - minimalistic laser visualization on Hildon
 *
 *  Created: Sun Oct 12 17:06:06 2008
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

#include "laser_drawing_area.h"

#include <gui_utils/robot/allemaniacs_athome.h>
#include <gtkmm.h>
#include <hildonmm.h>
#include <libosso.h>

#include <netcomm/fawkes/client.h>
#include <blackboard/remote.h>
#include <interfaces/Laser360Interface.h>
#include <gui_utils/interface_dispatcher.h>
#include <gui_utils/connection_dispatcher.h>
#include <gui_utils/service_chooser_dialog.h>

#if MAEMO_VERSION_MAJOR >= 5
#  define ICON_FORMAT "white_48x48"
#else
#  define ICON_FORMAT "32x32"
#endif

using namespace fawkes;

/** @class LaserGuiHildonWindow "lasergui_hildon.cpp"
 * Laser GUI window for Hildon.
 * @author Tim Niemueller
 */

class LaserGuiHildonWindow : public Hildon::Window
{
 public:
  /** Constructor. */
  LaserGuiHildonWindow()
    : __athome_drawer(true),
      __img_lines(RESDIR"/guis/lasergui/lines_"ICON_FORMAT".png"),
      __img_points(RESDIR"/guis/lasergui/points_"ICON_FORMAT".png"),
      __img_hull(RESDIR"/guis/lasergui/hull_"ICON_FORMAT".png"),
      __img_lowres(RESDIR"/guis/lasergui/lines_lowres_"ICON_FORMAT".png"),
      __img_rotation(RESDIR"/guis/lasergui/rotate-90.png"),
      __tb_connection(Gtk::Stock::CONNECT),
      __tb_lines(__img_lines),
      __tb_points(__img_points),
      __tb_hull(__img_hull),
      __tb_lowres(__img_lowres),
      __tb_rotation(__img_rotation),
      __tb_zoom_in(Gtk::Stock::ZOOM_IN),
      __tb_zoom_out(Gtk::Stock::ZOOM_OUT)
  {
    __fullscreen = false;
    __bb = NULL;
    __laser_if = NULL;
    __ifd = NULL;

#  if GLIBMM_MAJOR_VERSION > 2 || (GLIBMM_MAJOR_VERSION == 2 && GLIBMM_MINOR_VERSION >= 48)
    std::unique_ptr<Glib::Error> error;
#  else
    std::auto_ptr<Glib::Error> error;
#  endif
    set_icon_from_file(RESDIR"/guis/lasergui/lines_"ICON_FORMAT".png", error);

    add(__area);
    __area.show();
    __area.set_robot_drawer(&__athome_drawer);

    Gtk::RadioButton::Group group = __tb_lines.get_group();
    __tb_points.set_group(group);
    group = __tb_lines.get_group();
    __tb_hull.set_group(group);
    __tb_lines.set_active(true);

    __tb_lines.set_sensitive(false);
    __tb_points.set_sensitive(false);
    __tb_hull.set_sensitive(false);
    __tb_lowres.set_sensitive(false);
    __tb_rotation.set_sensitive(false);
    __tb_zoom_in.set_sensitive(false);
    __tb_zoom_out.set_sensitive(false);

    __tbar.append(__tb_connection);
    __tbar.append(__sep_0);
    __tbar.append(__tb_lines);
    __tbar.append(__tb_points);
    __tbar.append(__tb_hull);
    __tbar.append(__sep_1);
    __tbar.append(__tb_lowres);
    __tbar.append(__tb_rotation);
    __tbar.append(__sep_2);
    __tbar.append(__tb_zoom_in);
    __tbar.append(__tb_zoom_out);

    add_toolbar(__tbar);
    __tbar.show_all();

    __tb_lines.signal_toggled().connect(sigc::bind(sigc::mem_fun(__area, &LaserDrawingArea::set_draw_mode), LaserDrawingArea::MODE_LINES));
    __tb_points.signal_toggled().connect(sigc::bind(sigc::mem_fun(__area, &LaserDrawingArea::set_draw_mode), LaserDrawingArea::MODE_POINTS));
    __tb_hull.signal_toggled().connect(sigc::bind(sigc::mem_fun(__area, &LaserDrawingArea::set_draw_mode), LaserDrawingArea::MODE_HULL));
    __tb_zoom_in.signal_clicked().connect(sigc::mem_fun(__area, &LaserDrawingArea::zoom_in));
    __tb_zoom_out.signal_clicked().connect(sigc::mem_fun(__area, &LaserDrawingArea::zoom_out));

    __tb_connection.signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiHildonWindow::on_connection_clicked));
    __tb_rotation.signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiHildonWindow::on_rotation_toggled));
    __tb_lowres.signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiHildonWindow::on_resolution_toggled));

    __connection_dispatcher.signal_connected().connect(sigc::mem_fun(*this, &LaserGuiHildonWindow::on_connect));
    __connection_dispatcher.signal_disconnected().connect(sigc::mem_fun(*this, &LaserGuiHildonWindow::on_disconnect));

#ifndef GLIBMM_DEFAULT_SIGNAL_HANDLERS_ENABLED
    signal_key_press_event().connect(sigc::mem_fun(*this, &LaserGuiHildonWindow::on_key_pressed));
    signal_window_state_event().connect(sigc::mem_fun(*this, &LaserGuiHildonWindow::on_window_state_event));
#endif
  }

  /** Destructor. */
  ~LaserGuiHildonWindow()
  {
    __area.set_laser360_if(NULL);
    if (__bb) {
      __bb->close(__laser_if);
      delete __bb;
      delete __ifd;
    }
  }

 protected:
  /** Event handler for key pressed events.
   * @param event event parameters
   * @return always false
   */
  virtual bool on_key_pressed(GdkEventKey* event)
  {
    if(!event)  return false;

    switch (event->keyval) {
    case GDK_F6:
      if ( __fullscreen ) {
	unfullscreen();
      } else {
	fullscreen();
      }
      break;
    case GDK_F7:
      __area.zoom_in();
      break;
    case GDK_F8:
      __area.zoom_out();
      break;
    }

    // Returning true would stop the event now
    return false;
  }

  /** Event handler for window state change events.
   * @param event event parameters
   * @return always false
   */
  virtual bool on_window_state_event(GdkEventWindowState *event)
  {
    if (event->new_window_state == GDK_WINDOW_STATE_FULLSCREEN) {
      __fullscreen = true;
    } else {
      __fullscreen = false;
    }
    return false;
  }

  /** Event handler for connection button. */
  void on_connection_clicked()
  {
    if ( ! __connection_dispatcher.get_client()->connected() ) {
      ServiceChooserDialog ssd(*this, __connection_dispatcher.get_client());
      ssd.run_and_connect();
    } else {
      __connection_dispatcher.get_client()->disconnect();
    }
  }

  /** Event handler for connected event. */
  virtual void on_connect()
  {
    try {
      __bb = new RemoteBlackBoard(__connection_dispatcher.get_client());
      __laser_if = __bb->open_for_reading<Laser360Interface>("Laser");

      __area.set_laser360_if(__laser_if);
      __ifd = new InterfaceDispatcher("LaserInterfaceDispatcher", __laser_if);
      __ifd->signal_data_changed().connect(sigc::hide(sigc::mem_fun(__area, &LaserDrawingArea::queue_draw)));
      __ifd->signal_writer_removed().connect(sigc::hide(sigc::mem_fun(__area, &LaserDrawingArea::queue_draw)));
      __bb->register_listener(__ifd, BlackBoard::BBIL_FLAG_DATA | BlackBoard::BBIL_FLAG_WRITER);

      __area.queue_draw();

      __tb_connection.set_stock_id(Gtk::Stock::DISCONNECT);
      __tb_lines.set_sensitive(true);
      __tb_points.set_sensitive(true);
      __tb_hull.set_sensitive(true);
      __tb_lowres.set_sensitive(true);
      __tb_rotation.set_sensitive(true);
      __tb_zoom_in.set_sensitive(true);
      __tb_zoom_out.set_sensitive(true);
    } catch (Exception &e) {
      e.print_trace();
      if ( __bb ) {
	__bb->close(__laser_if);
	delete __ifd;
	delete __bb;
	__laser_if = NULL;
	__bb = NULL;
	__ifd = NULL;
      }
    }
  }

  /** Event handler for disconnected event. */
  virtual void on_disconnect()
  {
    __area.set_laser360_if(NULL);
    __area.queue_draw();
    __bb->close(__laser_if);
    delete __bb;
    delete __ifd;
    __bb = NULL;
    __ifd = NULL;
    __laser_if = NULL;
    __tb_connection.set_stock_id(Gtk::Stock::CONNECT);
    __tb_lines.set_sensitive(false);
    __tb_points.set_sensitive(false);
    __tb_hull.set_sensitive(false);
    __tb_lowres.set_sensitive(false);
    __tb_rotation.set_sensitive(false);
    __tb_zoom_in.set_sensitive(false);
    __tb_zoom_out.set_sensitive(false);
  }

  /** Event handler for rotation button. */
  void on_rotation_toggled()
  {
    if ( __tb_rotation.get_active() ) {
      __area.set_rotation(M_PI / 2);
    } else {
      __area.set_rotation(0);
    }
  }

  /** Event handler for rotation button. */
  void on_resolution_toggled()
  {
    if ( __tb_lowres.get_active() ) {
      __area.set_resolution(3);
    } else {
      __area.set_resolution(1);
    }
  }

 private:
  AllemaniACsAtHomeCairoRobotDrawer  __athome_drawer;
  BlackBoard                        *__bb;
  Laser360Interface                 *__laser_if;
  InterfaceDispatcher               *__ifd;
  ConnectionDispatcher               __connection_dispatcher;

  Gtk::Image                         __img_lines;
  Gtk::Image                         __img_points;
  Gtk::Image                         __img_hull;
  Gtk::Image                         __img_lowres;
  Gtk::Image                         __img_rotation;
  Gtk::ToolButton                    __tb_connection;
  Gtk::SeparatorToolItem             __sep_0;
  Gtk::RadioToolButton               __tb_lines;
  Gtk::RadioToolButton               __tb_points;
  Gtk::RadioToolButton               __tb_hull;
  Gtk::SeparatorToolItem             __sep_1;
  Gtk::ToggleToolButton              __tb_lowres;
  Gtk::ToggleToolButton              __tb_rotation;
  Gtk::SeparatorToolItem             __sep_2;
  Gtk::ToolButton                    __tb_zoom_in;
  Gtk::ToolButton                    __tb_zoom_out;
  Gtk::Toolbar                       __tbar;

  LaserDrawingArea                   __area;

  bool                               __fullscreen;
};

int
main(int argc, char** argv)
{
  Gtk::Main kit(argc, argv);
  Hildon::init();

  osso_context_t* osso_context = osso_initialize("lasergui", "0.1", TRUE /* deprecated parameter */, 0 /* Use default Glib main loop context */);
  Glib::set_application_name("Laser GUI");

  LaserGuiHildonWindow window;
  kit.run(window);

  osso_deinitialize(osso_context);
  return 0;
}
