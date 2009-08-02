
/***************************************************************************
 *  lasergui.cpp - minimalistic laser visualization
 *
 *  Created: Thu Oct 09 12:51:52 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *             2009  Masrur Doostdar <doostdar@kbsg.rwth-aachen.de>
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

#include "laser_drawing_area.h"

#include <netcomm/fawkes/client.h>
#include <blackboard/remote.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>

#include <interfaces/ObjectPositionInterface.h>
#include <interfaces/Position2DTrackInterface.h>
#include <interfaces/SwitchInterface.h>


#include <gui_utils/service_chooser_dialog.h>
#include <gui_utils/interface_dispatcher.h>
#include <gui_utils/connection_dispatcher.h>
#include <gui_utils/robot/allemaniacs_athome.h>

#include <gtkmm/main.h>
#include <libglademm/xml.h>
#include <list>
#include <utils/misc/string_conversions.h>


#define MAX_OBJECTPOSITIONINTERFACES_PERSONS 10
#define MAX_OBJECTPOSITIONINTERFACES_LEGS 15
#define MAX_OBJECTPOSITIONINTERFACES_MISC 20
#define MAX_TRACKINTERFACES 10

using namespace fawkes;

/** @class LaserGuiGtkWindow "lasergui.cpp"
 * Laser GUI window for Gtkmm.
 * @author Tim Niemueller
 */
class LaserGuiGtkWindow : public Gtk::Window
{
 public:
  /** Constructor for Glademm.
   * @param cobject C base object
   * @param refxml reference to Glade's Xml parser
   */
  LaserGuiGtkWindow(BaseObjectType* cobject,
		    const Glib::RefPtr<Gnome::Glade::Xml> &refxml)
    : Gtk::Window(cobject), __athome_drawer(true)
  {
    refxml->get_widget_derived("da_laser", __area);
    refxml->get_widget("tb_connection", __tb_connection);
    refxml->get_widget("tb_lines", __tb_lines);
    refxml->get_widget("tb_points", __tb_points);
    refxml->get_widget("tb_hull", __tb_hull);
    refxml->get_widget("tb_highres", __tb_highres);
    refxml->get_widget("tb_trimvals", __tb_trimvals);
    refxml->get_widget("tb_rotation", __tb_rotation);
    refxml->get_widget("tb_legtracker", __tb_legtracker);
    refxml->get_widget("tb_stop", __tb_stop);
    refxml->get_widget("tb_zoom_in", __tb_zoom_in);
    refxml->get_widget("tb_zoom_out", __tb_zoom_out);
    refxml->get_widget("tb_exit", __tb_exit);
    refxml->get_widget("dlg_ltopen", __dlg_ltopen);
    refxml->get_widget("pgb_ltopen", __pgb_ltopen);

    __area->set_robot_drawer(&__athome_drawer);

    __tb_lines->set_sensitive(false);
    __tb_points->set_sensitive(false);
    __tb_hull->set_sensitive(false);
    __tb_highres->set_sensitive(false);
    __tb_trimvals->set_sensitive(false);
    __tb_rotation->set_sensitive(false);
    __tb_legtracker->set_sensitive(false);
    __tb_stop->set_sensitive(false);
    __tb_zoom_in->set_sensitive(false);
    __tb_zoom_out->set_sensitive(false);

    __tb_connection->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_connection_clicked));
    __tb_lines->signal_toggled().connect(sigc::bind(sigc::mem_fun(*__area, &LaserDrawingArea::set_draw_mode), LaserDrawingArea::MODE_LINES));
    __tb_points->signal_toggled().connect(sigc::bind(sigc::mem_fun(*__area, &LaserDrawingArea::set_draw_mode), LaserDrawingArea::MODE_POINTS));
    __tb_hull->signal_toggled().connect(sigc::bind(sigc::mem_fun(*__area, &LaserDrawingArea::set_draw_mode), LaserDrawingArea::MODE_HULL));
    __tb_zoom_in->signal_clicked().connect(sigc::mem_fun(*__area, &LaserDrawingArea::zoom_in));
    __tb_zoom_out->signal_clicked().connect(sigc::mem_fun(*__area, &LaserDrawingArea::zoom_out));

    __tb_highres->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_resolution_toggled));
    __tb_legtracker->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_legtracker_toggled));
    __tb_trimvals->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_trimvals_toggled));
    __tb_rotation->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_rotation_toggled));
    __tb_stop->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_stop_toggled));
    __tb_exit->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_exit_clicked));

    __connection_dispatcher.signal_connected().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_connect));
    __connection_dispatcher.signal_disconnected().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_disconnect));
  }


 protected:
  /** Event handler for connection button. */
  virtual void on_connection_clicked()
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
      __laser360_if = NULL;
      __laser720_if = NULL;
      __l_objpos_if_persons = NULL;
      __l_objpos_if_legs = NULL;
      __l_objpos_if_misc = NULL;
      __l_track_if = NULL;
      __laser_segmentation_if = NULL;
      __switch_if = NULL;
      __target_if = NULL;
      __line_if = NULL;
      
      //__laser_if = __bb->open_for_reading<Laser360Interface>("LegtrackerAveragedLaser");


      if (__tb_highres->get_active()) {
	__laser720_if = __bb->open_for_reading<Laser720Interface>("Laser");
	__area->set_laser720_if(__laser720_if);
	__ifd = new InterfaceDispatcher("LaserInterfaceDispatcher", __laser720_if);
      } else {
	__laser360_if = __bb->open_for_reading<Laser360Interface>("Laser");
	__area->set_laser360_if(__laser360_if);
	__ifd = new InterfaceDispatcher("LaserInterfaceDispatcher", __laser360_if);
      }

      __line_if = __bb->open_for_reading<ObjectPositionInterface>("LaserLine");
      __area->set_line_if(__line_if);

      on_legtracker_toggled();

      __ifd->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*__area, &LaserDrawingArea::queue_draw)));
      __bb->register_listener(__ifd, BlackBoard::BBIL_FLAG_DATA);

      __area->queue_draw();

      __tb_connection->set_stock_id(Gtk::Stock::DISCONNECT);
      __tb_lines->set_sensitive(true);
      __tb_points->set_sensitive(true);
      __tb_hull->set_sensitive(true);
      __tb_highres->set_sensitive(true);
      __tb_trimvals->set_sensitive(true);
      __tb_rotation->set_sensitive(true);
      __tb_legtracker->set_sensitive(true);
      __tb_stop->set_sensitive(true);
      __tb_zoom_in->set_sensitive(true);
      __tb_zoom_out->set_sensitive(true);
    } catch (Exception &e) {
      if ( __bb ) {
	__bb->close(__laser360_if);
	__bb->close(__laser720_if);
	__bb->close(__line_if);
	delete __ifd;
	delete __bb;
	__laser360_if = NULL;
	__laser720_if = NULL;
	__bb = NULL;
	__ifd = NULL;
	__line_if = NULL;
      }
    }
  }

  /** Event handler for disconnected event. */
  virtual void on_disconnect()
  {
    __area->reset_laser_ifs();
    __area->set_line_if(NULL);
    __area->queue_draw();
    if(__laser360_if)
      __bb->close(__laser360_if);
    if(__laser720_if)
      __bb->close(__laser720_if);
    if(__laser_segmentation_if)
      __bb->close(__laser_segmentation_if);
    if(__switch_if)
      __bb->close(__switch_if);
    if(__target_if)
      __bb->close(__target_if);
    __bb->close(__line_if);

    std::list<ObjectPositionInterface*>::iterator objpos_if_itt;
    std::list<Position2DTrackInterface*>::iterator track_if_itt;
    if(__l_objpos_if_persons){
      for( objpos_if_itt = __l_objpos_if_persons->begin(); objpos_if_itt != __l_objpos_if_persons->end(); objpos_if_itt++ ) {
	__bb->close(*objpos_if_itt);
      }
      __l_objpos_if_persons->clear();
    }
    if(__l_objpos_if_legs){
      for( objpos_if_itt = __l_objpos_if_legs->begin(); objpos_if_itt != __l_objpos_if_legs->end(); objpos_if_itt++ ) {
	__bb->close(*objpos_if_itt);
      }
      __l_objpos_if_legs->clear();
    }
    if(__l_objpos_if_misc){
      for( objpos_if_itt = __l_objpos_if_misc->begin(); objpos_if_itt != __l_objpos_if_misc->end(); objpos_if_itt++ ) {
	__bb->close(*objpos_if_itt);
      }
      __l_objpos_if_misc->clear();
    }
    if(__l_track_if){
      for( track_if_itt = __l_track_if->begin(); track_if_itt != __l_track_if->end(); track_if_itt++ ) {
	__bb->close(*track_if_itt);
      }
      __l_track_if->clear();
    }



    delete __bb;
    delete __ifd;
    __bb = NULL;
    __ifd = NULL;
    __laser360_if = NULL;
    __laser720_if = NULL;
    __l_objpos_if_persons = NULL;
    __l_objpos_if_legs = NULL;
    __l_objpos_if_misc = NULL;
    __l_track_if = NULL;
    __laser_segmentation_if = NULL;
    __switch_if = NULL;
    __target_if = NULL;
    __line_if = NULL;

    __tb_connection->set_stock_id(Gtk::Stock::CONNECT);
    __tb_lines->set_sensitive(false);
    __tb_points->set_sensitive(false);
    __tb_hull->set_sensitive(false);
    __tb_highres->set_sensitive(false);
    __tb_trimvals->set_sensitive(false);
    __tb_rotation->set_sensitive(false);
    __tb_legtracker->set_sensitive(false);
    __tb_stop->set_sensitive(false);
    __tb_zoom_in->set_sensitive(false);
    __tb_zoom_out->set_sensitive(false);
  }

  
  /** Event handler for rotation button. */
  void on_rotation_toggled()
  {
    if ( __tb_rotation->get_active() ) {
      __area->set_rotation(M_PI / 2);
    } else {
      __area->set_rotation(0);
    }
  }
  

  /** Event handler for stop button */
  void on_stop_toggled()
  {
    __area->toggle_break_drawing();
  }

  /** Event handler for resolution button. */
  void on_resolution_toggled()
  {
    if (! __bb)  return;

    try {
      __bb->close(__laser360_if);
      __bb->close(__laser720_if);
      __bb->unregister_listener(__ifd);
      delete __ifd;
      __laser360_if = NULL;
      __laser720_if = NULL;

      if ( __tb_highres->get_active() ) {
	__laser720_if = __bb->open_for_reading<Laser720Interface>("Laser");
	__ifd = new InterfaceDispatcher("LaserInterfaceDispatcher", __laser720_if);
	__ifd->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*__area, &LaserDrawingArea::queue_draw)));
	__bb->register_listener(__ifd, BlackBoard::BBIL_FLAG_DATA);
	__area->set_laser720_if(__laser720_if);
      } else {
	__laser360_if = __bb->open_for_reading<Laser360Interface>("Laser");
	__ifd = new InterfaceDispatcher("LaserInterfaceDispatcher", __laser360_if);
	__ifd->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*__area, &LaserDrawingArea::queue_draw)));
	__bb->register_listener(__ifd, BlackBoard::BBIL_FLAG_DATA);
	__area->set_laser360_if(__laser360_if);
      }
      __area->queue_draw();
    } catch (Exception &e) {
      e.print_trace();
    }
  }

  /** Event handler for legtracker button */
  void on_legtracker_toggled()
  {
    if (! __bb)  return;

    if (!__tb_legtracker->get_active()) {
      __bb->close(__laser_segmentation_if);
      __bb->close(__switch_if);
      __bb->close(__target_if);

      std::list<ObjectPositionInterface*>::iterator objpos_if_itt;
      std::list<Position2DTrackInterface*>::iterator track_if_itt;
      if (__l_objpos_if_persons) {
	for( objpos_if_itt = __l_objpos_if_persons->begin(); objpos_if_itt != __l_objpos_if_persons->end(); objpos_if_itt++ ) {
	  __bb->close(*objpos_if_itt);
	}
	__l_objpos_if_persons->clear();
      }
      if (__l_objpos_if_legs) {
	for( objpos_if_itt = __l_objpos_if_legs->begin(); objpos_if_itt != __l_objpos_if_legs->end(); objpos_if_itt++ ) {
	  __bb->close(*objpos_if_itt);
	}
	__l_objpos_if_legs->clear();
      }
      if (__l_objpos_if_misc) {
	for( objpos_if_itt = __l_objpos_if_misc->begin(); objpos_if_itt != __l_objpos_if_misc->end(); objpos_if_itt++ ) {
	  __bb->close(*objpos_if_itt);
	}
	__l_objpos_if_misc->clear();
      }

      if (__l_track_if) {
	for( track_if_itt = __l_track_if->begin(); track_if_itt != __l_track_if->end(); track_if_itt++ ) {
	  __bb->close(*track_if_itt);
	}
	__l_track_if->clear();
      }
      
      __laser_segmentation_if = NULL;
      __switch_if = NULL;
      __target_if = NULL;
      __l_objpos_if_persons = NULL;
      __l_objpos_if_legs = NULL;
      __l_objpos_if_misc = NULL;
      __l_track_if = NULL;

      __area->set_objpos_if(__l_objpos_if_persons,__l_objpos_if_legs,__l_objpos_if_misc,__laser_segmentation_if, __l_track_if, __target_if,__switch_if);

    } else {
      unsigned int num_opens = 3
	+ MAX_OBJECTPOSITIONINTERFACES_PERSONS
	+ MAX_OBJECTPOSITIONINTERFACES_LEGS
	+ MAX_OBJECTPOSITIONINTERFACES_MISC
	+ MAX_TRACKINTERFACES;

      float step_fraction = 1.0 / num_opens;
      unsigned int opened = 0;
      __pgb_ltopen->set_fraction(0);
      __dlg_ltopen->show();
      __area->queue_draw();

      __laser_segmentation_if = __bb->open_for_reading<Laser720Interface>("SegmentsLaser");
      __pgb_ltopen->set_fraction(++opened * step_fraction);
      while (Gtk::Main::events_pending()) Gtk::Main::iteration();

      __target_if = __bb->open_for_reading<ObjectPositionInterface>("legtracker Target");

      ObjectPositionInterface* new_objpos_if;
      __l_objpos_if_persons = new std::list<ObjectPositionInterface*>();
      __l_objpos_if_legs = new std::list<ObjectPositionInterface*>();
      __l_objpos_if_misc = new std::list<ObjectPositionInterface*>();
      __l_track_if = new std::list<Position2DTrackInterface*>();
      for (int i=1; i <= MAX_OBJECTPOSITIONINTERFACES_PERSONS; ++i){
	new_objpos_if= __bb->open_for_reading<ObjectPositionInterface>((std::string("legtracker CurrentLegsTracked") + StringConversions::to_string(i)).c_str());
	__l_objpos_if_persons->push_back(new_objpos_if);
	__pgb_ltopen->set_fraction(++opened * step_fraction);
	while (Gtk::Main::events_pending()) Gtk::Main::iteration();
      }
      for (int i=1; i <= MAX_OBJECTPOSITIONINTERFACES_LEGS; ++i){
	new_objpos_if= __bb->open_for_reading<ObjectPositionInterface>((std::string("legtracker Leg") + StringConversions::to_string(i)).c_str());
	__l_objpos_if_legs->push_back(new_objpos_if);
	__pgb_ltopen->set_fraction(++opened * step_fraction);
	while (Gtk::Main::events_pending()) Gtk::Main::iteration();
      }
      for (int i=1; i <= MAX_OBJECTPOSITIONINTERFACES_MISC; ++i){
	new_objpos_if= __bb->open_for_reading<ObjectPositionInterface>((std::string("legtracker Misc") + StringConversions::to_string(i)).c_str());
	__l_objpos_if_misc->push_back(new_objpos_if);
	__pgb_ltopen->set_fraction(++opened * step_fraction);
	while (Gtk::Main::events_pending()) Gtk::Main::iteration();
      }
      Position2DTrackInterface* new_track_if;
      for (int i=1; i <= MAX_TRACKINTERFACES; ++i){
	new_track_if = __bb->open_for_reading<Position2DTrackInterface>((std::string("legtracker Track") + StringConversions::to_string(i)).c_str());
	__l_track_if->push_back( new_track_if );
	__pgb_ltopen->set_fraction(++opened * step_fraction);
	while (Gtk::Main::events_pending()) Gtk::Main::iteration();
      }
      
      __switch_if = __bb->open_for_reading<SwitchInterface>("legtracker write!");
      __pgb_ltopen->set_fraction(++opened * step_fraction);
      while (Gtk::Main::events_pending()) Gtk::Main::iteration();
      __dlg_ltopen->hide();
      __area->set_objpos_if(__l_objpos_if_persons, __l_objpos_if_legs,
			    __l_objpos_if_misc,__laser_segmentation_if,
			    __l_track_if, __target_if,__switch_if);
      __area->queue_draw();
    }
  }


  /** Event handler for trim button. */
  void on_trimvals_toggled()
  {
    if ( __tb_trimvals->get_active() ) {
      __area->set_resolution(3);
    } else {
      __area->set_resolution(1);
    }
  }

  /** Event handler for exit button. */
  void on_exit_clicked()
  {
    Gtk::Main::quit();
  }

 private:
  BlackBoard                        *__bb;
  Laser360Interface                 *__laser360_if;
  Laser720Interface                 *__laser720_if;
  Laser720Interface                 *__laser_segmentation_if;
  SwitchInterface                   *__switch_if;
  ObjectPositionInterface           *__target_if;
  
  InterfaceDispatcher               *__ifd;
  std::list<ObjectPositionInterface*>* __l_objpos_if_persons;
  std::list<ObjectPositionInterface*>* __l_objpos_if_legs;
  std::list<ObjectPositionInterface*>* __l_objpos_if_misc;
  std::list<Position2DTrackInterface*>* __l_track_if;

  ObjectPositionInterface            *__line_if;

  LaserDrawingArea                   *__area;
  AllemaniACsAtHomeCairoRobotDrawer   __athome_drawer;
  ConnectionDispatcher                __connection_dispatcher;

  Gtk::ToolButton                    *__tb_connection;
  Gtk::RadioToolButton               *__tb_lines;
  Gtk::RadioToolButton               *__tb_points;
  Gtk::RadioToolButton               *__tb_hull;
  Gtk::ToggleToolButton              *__tb_highres;
  Gtk::ToggleToolButton              *__tb_trimvals;
  Gtk::ToggleToolButton              *__tb_rotation;
  Gtk::ToggleToolButton              *__tb_legtracker;
  Gtk::ToggleToolButton              *__tb_stop;
  Gtk::ToolButton                    *__tb_zoom_in;
  Gtk::ToolButton                    *__tb_zoom_out;
  Gtk::ToolButton                    *__tb_exit;

  Gtk::Dialog                        *__dlg_ltopen;
  Gtk::ProgressBar                   *__pgb_ltopen;
};

int
main(int argc, char** argv)
{
   Gtk::Main kit(argc, argv);
   
   Glib::RefPtr<Gnome::Glade::Xml> refxml;
   refxml = Gnome::Glade::Xml::create(RESDIR"/lasergui/lasergui.glade");

   LaserGuiGtkWindow *window = NULL;
   refxml->get_widget_derived("wnd_lasergui", window);

   Gtk::Main::run(*window);

   return 0;
}
