
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

#include <blackboard/remote.h>
#include <gui_utils/connection_dispatcher.h>
#include <gui_utils/interface_dispatcher.h>
#include <gui_utils/robot/allemaniacs_athome.h>
#include <gui_utils/service_chooser_dialog.h>
#include <interfaces/Laser360Interface.h>
#include <netcomm/fawkes/client.h>

#include <gtkmm.h>
#include <hildonmm.h>
#include <libosso.h>

#if MAEMO_VERSION_MAJOR >= 5
#	define ICON_FORMAT "white_48x48"
#else
#	define ICON_FORMAT "32x32"
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
	: athome_drawer_(true),
	  img_lines_(RESDIR "/guis/lasergui/lines_" ICON_FORMAT ".png"),
	  img_points_(RESDIR "/guis/lasergui/points_" ICON_FORMAT ".png"),
	  img_hull_(RESDIR "/guis/lasergui/hull_" ICON_FORMAT ".png"),
	  img_lowres_(RESDIR "/guis/lasergui/lines_lowres_" ICON_FORMAT ".png"),
	  img_rotation_(RESDIR "/guis/lasergui/rotate-90.png"),
	  tb_connection_(Gtk::Stock::CONNECT),
	  tb_lines_(img_lines_),
	  tb_points_(img_points_),
	  tb_hull_(img_hull_),
	  tb_lowres_(img_lowres_),
	  tb_rotation_(img_rotation_),
	  tb_zoom_in_(Gtk::Stock::ZOOM_IN),
	  tb_zoom_out_(Gtk::Stock::ZOOM_OUT)
	{
		fullscreen_ = false;
		bb_         = NULL;
		laser_if_   = NULL;
		ifd_        = NULL;

#if GLIBMM_MAJOR_VERSION > 2 || (GLIBMM_MAJOR_VERSION == 2 && GLIBMM_MINOR_VERSION >= 48)
		std::unique_ptr<Glib::Error> error;
#else
		std::auto_ptr<Glib::Error> error;
#endif
		set_icon_from_file(RESDIR "/guis/lasergui/lines_" ICON_FORMAT ".png", error);

		add(area_);
		area_.show();
		area_.set_robot_drawer(&athome_drawer_);

		Gtk::RadioButton::Group group = tb_lines_.get_group();
		tb_points_.set_group(group);
		group = tb_lines_.get_group();
		tb_hull_.set_group(group);
		tb_lines_.set_active(true);

		tb_lines_.set_sensitive(false);
		tb_points_.set_sensitive(false);
		tb_hull_.set_sensitive(false);
		tb_lowres_.set_sensitive(false);
		tb_rotation_.set_sensitive(false);
		tb_zoom_in_.set_sensitive(false);
		tb_zoom_out_.set_sensitive(false);

		tbar_.append(tb_connection_);
		tbar_.append(sep_0_);
		tbar_.append(tb_lines_);
		tbar_.append(tb_points_);
		tbar_.append(tb_hull_);
		tbar_.append(sep_1_);
		tbar_.append(tb_lowres_);
		tbar_.append(tb_rotation_);
		tbar_.append(sep_2_);
		tbar_.append(tb_zoom_in_);
		tbar_.append(tb_zoom_out_);

		add_toolbar(tbar_);
		tbar_.show_all();

		tb_lines_.signal_toggled().connect(
		  sigc::bind(sigc::mem_fun(area_, &LaserDrawingArea::set_draw_mode),
		             LaserDrawingArea::MODE_LINES));
		tb_points_.signal_toggled().connect(
		  sigc::bind(sigc::mem_fun(area_, &LaserDrawingArea::set_draw_mode),
		             LaserDrawingArea::MODE_POINTS));
		tb_hull_.signal_toggled().connect(
		  sigc::bind(sigc::mem_fun(area_, &LaserDrawingArea::set_draw_mode),
		             LaserDrawingArea::MODE_HULL));
		tb_zoom_in_.signal_clicked().connect(sigc::mem_fun(area_, &LaserDrawingArea::zoom_in));
		tb_zoom_out_.signal_clicked().connect(sigc::mem_fun(area_, &LaserDrawingArea::zoom_out));

		tb_connection_.signal_clicked().connect(
		  sigc::mem_fun(*this, &LaserGuiHildonWindow::on_connection_clicked));
		tb_rotation_.signal_clicked().connect(
		  sigc::mem_fun(*this, &LaserGuiHildonWindow::on_rotation_toggled));
		tb_lowres_.signal_clicked().connect(
		  sigc::mem_fun(*this, &LaserGuiHildonWindow::on_resolution_toggled));

		connection_dispatcher_.signal_connected().connect(
		  sigc::mem_fun(*this, &LaserGuiHildonWindow::on_connect));
		connection_dispatcher_.signal_disconnected().connect(
		  sigc::mem_fun(*this, &LaserGuiHildonWindow::on_disconnect));

#ifndef GLIBMM_DEFAULT_SIGNAL_HANDLERS_ENABLED
		signal_key_press_event().connect(sigc::mem_fun(*this, &LaserGuiHildonWindow::on_key_pressed));
		signal_window_state_event().connect(
		  sigc::mem_fun(*this, &LaserGuiHildonWindow::on_window_state_event));
#endif
	}

	/** Destructor. */
	~LaserGuiHildonWindow()
	{
		area_.set_laser360_if(NULL);
		if (bb_) {
			bb_->close(laser_if_);
			delete bb_;
			delete ifd_;
		}
	}

protected:
	/** Event handler for key pressed events.
   * @param event event parameters
   * @return always false
   */
	virtual bool
	on_key_pressed(GdkEventKey *event)
	{
		if (!event)
			return false;

		switch (event->keyval) {
		case GDK_F6:
			if (fullscreen_) {
				unfullscreen();
			} else {
				fullscreen();
			}
			break;
		case GDK_F7: area_.zoom_in(); break;
		case GDK_F8: area_.zoom_out(); break;
		}

		// Returning true would stop the event now
		return false;
	}

	/** Event handler for window state change events.
   * @param event event parameters
   * @return always false
   */
	virtual bool
	on_window_state_event(GdkEventWindowState *event)
	{
		if (event->new_window_state == GDK_WINDOW_STATE_FULLSCREEN) {
			fullscreen_ = true;
		} else {
			fullscreen_ = false;
		}
		return false;
	}

	/** Event handler for connection button. */
	void
	on_connection_clicked()
	{
		if (!connection_dispatcher_.get_client()->connected()) {
			ServiceChooserDialog ssd(*this, connection_dispatcher_.get_client());
			ssd.run_and_connect();
		} else {
			connection_dispatcher_.get_client()->disconnect();
		}
	}

	/** Event handler for connected event. */
	virtual void
	on_connect()
	{
		try {
			bb_       = new RemoteBlackBoard(connection_dispatcher_.get_client());
			laser_if_ = bb_->open_for_reading<Laser360Interface>("Laser");

			area_.set_laser360_if(laser_if_);
			ifd_ = new InterfaceDispatcher("LaserInterfaceDispatcher", laser_if_);
			ifd_->signal_data_changed().connect(
			  sigc::hide(sigc::mem_fun(area_, &LaserDrawingArea::queue_draw)));
			ifd_->signal_writer_removed().connect(
			  sigc::hide(sigc::mem_fun(area_, &LaserDrawingArea::queue_draw)));
			bb_->register_listener(ifd_, BlackBoard::BBIL_FLAG_DATA | BlackBoard::BBIL_FLAG_WRITER);

			area_.queue_draw();

			tb_connection_.set_stock_id(Gtk::Stock::DISCONNECT);
			tb_lines_.set_sensitive(true);
			tb_points_.set_sensitive(true);
			tb_hull_.set_sensitive(true);
			tb_lowres_.set_sensitive(true);
			tb_rotation_.set_sensitive(true);
			tb_zoom_in_.set_sensitive(true);
			tb_zoom_out_.set_sensitive(true);
		} catch (Exception &e) {
			e.print_trace();
			if (bb_) {
				bb_->close(laser_if_);
				delete ifd_;
				delete bb_;
				laser_if_ = NULL;
				bb_       = NULL;
				ifd_      = NULL;
			}
		}
	}

	/** Event handler for disconnected event. */
	virtual void
	on_disconnect()
	{
		area_.set_laser360_if(NULL);
		area_.queue_draw();
		bb_->close(laser_if_);
		delete bb_;
		delete ifd_;
		bb_       = NULL;
		ifd_      = NULL;
		laser_if_ = NULL;
		tb_connection_.set_stock_id(Gtk::Stock::CONNECT);
		tb_lines_.set_sensitive(false);
		tb_points_.set_sensitive(false);
		tb_hull_.set_sensitive(false);
		tb_lowres_.set_sensitive(false);
		tb_rotation_.set_sensitive(false);
		tb_zoom_in_.set_sensitive(false);
		tb_zoom_out_.set_sensitive(false);
	}

	/** Event handler for rotation button. */
	void
	on_rotation_toggled()
	{
		if (tb_rotation_.get_active()) {
			area_.set_rotation(M_PI / 2);
		} else {
			area_.set_rotation(0);
		}
	}

	/** Event handler for rotation button. */
	void
	on_resolution_toggled()
	{
		if (tb_lowres_.get_active()) {
			area_.set_resolution(3);
		} else {
			area_.set_resolution(1);
		}
	}

private:
	AllemaniACsAtHomeCairoRobotDrawer athome_drawer_;
	BlackBoard *                      bb_;
	Laser360Interface *               laser_if_;
	InterfaceDispatcher *             ifd_;
	ConnectionDispatcher              connection_dispatcher_;

	Gtk::Image             img_lines_;
	Gtk::Image             img_points_;
	Gtk::Image             img_hull_;
	Gtk::Image             img_lowres_;
	Gtk::Image             img_rotation_;
	Gtk::ToolButton        tb_connection_;
	Gtk::SeparatorToolItem sep_0_;
	Gtk::RadioToolButton   tb_lines_;
	Gtk::RadioToolButton   tb_points_;
	Gtk::RadioToolButton   tb_hull_;
	Gtk::SeparatorToolItem sep_1_;
	Gtk::ToggleToolButton  tb_lowres_;
	Gtk::ToggleToolButton  tb_rotation_;
	Gtk::SeparatorToolItem sep_2_;
	Gtk::ToolButton        tb_zoom_in_;
	Gtk::ToolButton        tb_zoom_out_;
	Gtk::Toolbar           tbar_;

	LaserDrawingArea area_;

	bool fullscreen_;
};

int
main(int argc, char **argv)
{
	Gtk::Main kit(argc, argv);
	Hildon::init();

	osso_context_t *osso_context = osso_initialize("lasergui",
	                                               "0.1",
	                                               TRUE /* deprecated parameter */,
	                                               0 /* Use default Glib main loop context */);
	Glib::set_application_name("Laser GUI");

	LaserGuiHildonWindow window;
	kit.run(window);

	osso_deinitialize(osso_context);
	return 0;
}
