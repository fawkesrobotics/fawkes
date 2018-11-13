
/***************************************************************************
 *  lasergui.cpp - minimalistic laser visualization
 *
 *  Created: Thu Oct 09 12:51:52 2008
 *  Copyright  2008-2011  Tim Niemueller [www.niemueller.de]
 *             2009       Masrur Doostdar <doostdar@kbsg.rwth-aachen.de>
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
#include <interfaces/Laser1080Interface.h>

#include <interfaces/ObjectPositionInterface.h>
#include <interfaces/Position2DTrackInterface.h>
#include <interfaces/SwitchInterface.h>
#include <interfaces/VisualDisplay2DInterface.h>


#include <gui_utils/service_chooser_dialog.h>
#include <gui_utils/multi_interface_chooser_dialog.h>
#include <gui_utils/interface_dispatcher.h>
#include <gui_utils/connection_dispatcher.h>
#include <gui_utils/robot/allemaniacs_athome.h>

#include <gtkmm/main.h>
#include <list>
#include <memory>
#include <set>
#include <map>
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
  /** Typedef of fawkes::Interface to override Glib::Interface. */
  typedef fawkes::Interface Interface;
  /** Shorthand for pair of interface type and ID. */
  typedef MultiInterfaceChooserDialog::TypeIdPair TypeIdPair;
  /** Shorthand for set of pairs of interface type and ID. */
  typedef MultiInterfaceChooserDialog::TypeIdPairList TypeIdPairList;
  /** For each interface, an interface dispatcher is opened that listens for
   * data changes. */
  typedef std::pair<Interface*, InterfaceDispatcher*> InterfaceDispatcherPair;
  /** A list of interfaces and their respective dispatchers.
   * Note that this is a list and not a map from interface to dispatcher only
   * to keep the ordering specified by the user in the GUI. */
  typedef std::list<InterfaceDispatcherPair> InterfaceDispatcherPairList;

  /** Constructor for Gtk::Builder.
   * @param cobject C base object
   * @param builder Gtk Builder
   */
  LaserGuiGtkWindow(BaseObjectType* cobject,
		    const Glib::RefPtr<Gtk::Builder> &builder)
    : Gtk::Window(cobject), athome_drawer_(true)
  {
    laser_if_names_.push_back(std::make_pair("Laser360Interface", "Laser"));

    builder->get_widget_derived("da_laser", area_);
    builder->get_widget("tb_connection", tb_connection_);
    builder->get_widget("tb_select", tb_select_);
    builder->get_widget("tb_lines", tb_lines_);
    builder->get_widget("tb_points", tb_points_);
    builder->get_widget("tb_hull", tb_hull_);
    builder->get_widget("tb_trimvals", tb_trimvals_);
    builder->get_widget("tb_rotation", tb_rotation_);
    builder->get_widget("tb_legtracker", tb_legtracker_);
    builder->get_widget("tb_stop", tb_stop_);
    builder->get_widget("tb_zoom_in", tb_zoom_in_);
    builder->get_widget("tb_zoom_out", tb_zoom_out_);
    builder->get_widget("tb_exit", tb_exit_);
    builder->get_widget("dlg_ltopen", dlg_ltopen_);
    builder->get_widget("pgb_ltopen", pgb_ltopen_);

    area_->set_robot_drawer(&athome_drawer_);

    tb_select_->set_sensitive(false);
    tb_lines_->set_sensitive(false);
    tb_points_->set_sensitive(false);
    tb_hull_->set_sensitive(false);
    tb_trimvals_->set_sensitive(false);
    tb_rotation_->set_sensitive(false);
    tb_legtracker_->set_sensitive(false);
    tb_stop_->set_sensitive(false);
    tb_zoom_in_->set_sensitive(false);
    tb_zoom_out_->set_sensitive(false);

    tb_connection_->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_connection_clicked));
    tb_select_->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_select_clicked));
    tb_lines_->signal_toggled().connect(sigc::bind(sigc::mem_fun(*area_, &LaserDrawingArea::set_draw_mode), LaserDrawingArea::MODE_LINES));
    tb_points_->signal_toggled().connect(sigc::bind(sigc::mem_fun(*area_, &LaserDrawingArea::set_draw_mode), LaserDrawingArea::MODE_POINTS));
    tb_hull_->signal_toggled().connect(sigc::bind(sigc::mem_fun(*area_, &LaserDrawingArea::set_draw_mode), LaserDrawingArea::MODE_HULL));
    tb_zoom_in_->signal_clicked().connect(sigc::mem_fun(*area_, &LaserDrawingArea::zoom_in));
    tb_zoom_out_->signal_clicked().connect(sigc::mem_fun(*area_, &LaserDrawingArea::zoom_out));

    tb_legtracker_->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_legtracker_toggled));
    tb_trimvals_->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_trimvals_toggled));
    tb_rotation_->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_rotation_toggled));
    tb_stop_->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_stop_toggled));
    tb_exit_->signal_clicked().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_exit_clicked));

    connection_dispatcher_.signal_connected().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_connect));
    connection_dispatcher_.signal_disconnected().connect(sigc::mem_fun(*this, &LaserGuiGtkWindow::on_disconnect));
  }


 protected:
  /** Event handler for connection button. */
  virtual void on_connection_clicked()
  {
    if ( ! connection_dispatcher_.get_client()->connected() ) {
      ServiceChooserDialog ssd(*this, connection_dispatcher_.get_client());
      ssd.run_and_connect();
    } else {
      connection_dispatcher_.get_client()->disconnect();
    }

  }

  /** Event handler for connection button. */
  virtual void on_select_clicked()
  {
    if ( ! connection_dispatcher_.get_client()->connected() ) {
      Gtk::MessageDialog md(*this,
			    "Cannot get list of interfaces if not connected.",
			    /* markup */ false,
			    Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			    /* modal */ true);
      md.set_title("Interface Selection Failed");
      md.run();
    } else {
#if __cplusplus >= 201103L
      std::unique_ptr<MultiInterfaceChooserDialog> ifcd(
#else
      std::auto_ptr<MultiInterfaceChooserDialog> ifcd(
#endif
          MultiInterfaceChooserDialog::create(*this,
                                              bb_,
                                              "Laser*Interface",
                                              "*",
                                              laser_if_names_));
      if (ifcd->run()) {
        const TypeIdPairList interfaces = ifcd->get_selected_interfaces();
        open_interfaces(interfaces);
      }
    }
  }

  /** Open interfaces.
   * Tries to open the interfaces.
   * Even if it fails, the old interfaces are closed.
   * @param types_and_ids types and ids of interfaces to open
   */
  void
  open_interfaces(const TypeIdPairList& types_and_ids)
  {
    area_->reset_laser_ifs();
    for (InterfaceDispatcherPairList::const_iterator it = laser_ifs_.begin();
         it != laser_ifs_.end(); ++it) {
      bb_->unregister_listener(it->second);
      delete it->second;
      bb_->close(it->first);
    }
    laser_ifs_.clear();
    laser_if_names_ = types_and_ids;

    // Open interfaces.
    for (TypeIdPairList::const_iterator it = types_and_ids.begin();
         it != types_and_ids.end(); ++it)
    {
      const Glib::ustring& type = it->first;
      const Glib::ustring& id = it->second;
      Interface* itf = NULL;
      try {
        if (type == "Laser1080Interface") {
          itf = bb_->open_for_reading<Laser1080Interface>(id.c_str());
        } else if (type == "Laser720Interface") {
          itf = bb_->open_for_reading<Laser720Interface>(id.c_str());
        } else if (type == "Laser360Interface") {
          itf = bb_->open_for_reading<Laser360Interface>(id.c_str());
        } else {
          throw Exception("Invalid interface type %s", type.c_str());
        }
      } catch (const Exception& e) {
        std::string msg = std::string("Failed to open interface: ") + e.what();
        Gtk::MessageDialog md(*this, msg, /* markup */ false,
                              Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
                              /* modal */ true);
        md.set_title("Opening Interface Failed");
        md.run();
        continue;
      }
      InterfaceDispatcher* itfd = new InterfaceDispatcher("LaserInterfaceDispatcher", itf);
      itfd->signal_data_changed().connect(
          sigc::hide(sigc::mem_fun(*area_, &LaserDrawingArea::queue_draw)));
      try {
        bb_->register_listener(itfd, BlackBoard::BBIL_FLAG_DATA);
      } catch (const Exception& e) {
        std::string msg = std::string("Failed to register interface dispatcher: ") + e.what();
        Gtk::MessageDialog md(*this, msg, /* markup */ false,
                              Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
                              /* modal */ true);
        md.set_title("Registrating Interface Dispatcher Failed");
        md.run();
        delete itfd;
        bb_->close(itf);
        continue;
      }
      const InterfaceDispatcherPair p = std::make_pair(itf, itfd);
      laser_ifs_.push_back(p);
    }

    // Inform the drawing area.
    std::list<Interface*> keys;
    for (InterfaceDispatcherPairList::const_iterator it = laser_ifs_.begin();
         it != laser_ifs_.end(); ++it)
    {
      keys.push_back(it->first);
    }
    area_->set_laser_ifs(keys);
  }

  /** Event handler for connected event. */
  virtual void on_connect()
  {
    try {
      bb_ = new RemoteBlackBoard(connection_dispatcher_.get_client());
      laser_ifs_.clear();
      l_objpos_if_persons_ = NULL;
      l_objpos_if_legs_ = NULL;
      l_objpos_if_misc_ = NULL;
      l_track_if_ = NULL;
      laser_segmentation_if_ = NULL;
      switch_if_ = NULL;
      target_if_ = NULL;
      line_if_ = NULL;
      visdis_if_ = NULL;
      
      //laser_if_ = bb_->open_for_reading<Laser360Interface>("LegtrackerAveragedLaser");

      area_->set_connected(true);
      open_interfaces(laser_if_names_);

      line_if_ = bb_->open_for_reading<ObjectPositionInterface>("LaserLine");
      area_->set_line_if(line_if_);
      try {
        visdis_if_ = bb_->open_for_writing<VisualDisplay2DInterface>("LaserGUI");
        area_->set_visdisp_if(visdis_if_);
      } catch (Exception &e) {
        visdis_if_ = NULL;
        // visdisplay is optional, probably some other lasergui has it
        // open atm
      }

      on_legtracker_toggled();

      area_->queue_draw();

      tb_connection_->set_stock_id(Gtk::Stock::DISCONNECT);
      tb_select_->set_sensitive(true);
      tb_lines_->set_sensitive(true);
      tb_points_->set_sensitive(true);
      tb_hull_->set_sensitive(true);
      tb_trimvals_->set_sensitive(true);
      tb_rotation_->set_sensitive(true);
      tb_legtracker_->set_sensitive(true);
      tb_stop_->set_sensitive(true);
      tb_zoom_in_->set_sensitive(true);
      tb_zoom_out_->set_sensitive(true);
    } catch (Exception &e) {
      area_->reset_laser_ifs();
      area_->set_line_if(NULL);
      area_->set_visdisp_if(NULL);
      area_->queue_draw();
      area_->set_connected(false);
      if ( bb_ ) {
        area_->reset_laser_ifs();
        for (InterfaceDispatcherPairList::const_iterator it = laser_ifs_.begin();
             it != laser_ifs_.end(); ++it) {
          bb_->unregister_listener(it->second);
          delete it->second;
          bb_->close(it->first);
        }
	bb_->close(line_if_);
	bb_->close(visdis_if_);
	delete bb_;
        laser_ifs_.clear();
	bb_ = NULL;
	line_if_ = NULL;
	visdis_if_ = NULL;
      }
    }
  }

  /** Event handler for disconnected event. */
  virtual void on_disconnect()
  {
    area_->set_connected(false);
    area_->reset_laser_ifs();
    area_->set_line_if(NULL);
    area_->set_visdisp_if(NULL);
    area_->queue_draw();
    for (InterfaceDispatcherPairList::const_iterator it = laser_ifs_.begin();
         it != laser_ifs_.end(); ++it) {
      bb_->unregister_listener(it->second);
      delete it->second;
      bb_->close(it->first);
    }
    laser_ifs_.clear();
    if(laser_segmentation_if_)
      bb_->close(laser_segmentation_if_);
    if(switch_if_)
      bb_->close(switch_if_);
    if(target_if_)
      bb_->close(target_if_);
    bb_->close(line_if_);
    bb_->close(visdis_if_);

    std::list<ObjectPositionInterface*>::iterator objpos_if_itt;
    std::list<Position2DTrackInterface*>::iterator track_if_itt;
    if(l_objpos_if_persons_){
      for( objpos_if_itt = l_objpos_if_persons_->begin(); objpos_if_itt != l_objpos_if_persons_->end(); objpos_if_itt++ ) {
	bb_->close(*objpos_if_itt);
      }
      l_objpos_if_persons_->clear();
    }
    if(l_objpos_if_legs_){
      for( objpos_if_itt = l_objpos_if_legs_->begin(); objpos_if_itt != l_objpos_if_legs_->end(); objpos_if_itt++ ) {
	bb_->close(*objpos_if_itt);
      }
      l_objpos_if_legs_->clear();
    }
    if(l_objpos_if_misc_){
      for( objpos_if_itt = l_objpos_if_misc_->begin(); objpos_if_itt != l_objpos_if_misc_->end(); objpos_if_itt++ ) {
	bb_->close(*objpos_if_itt);
      }
      l_objpos_if_misc_->clear();
    }
    if(l_track_if_){
      for( track_if_itt = l_track_if_->begin(); track_if_itt != l_track_if_->end(); track_if_itt++ ) {
	bb_->close(*track_if_itt);
      }
      l_track_if_->clear();
    }



    delete bb_;
    bb_ = NULL;
    laser_ifs_.clear();
    l_objpos_if_persons_ = NULL;
    l_objpos_if_legs_ = NULL;
    l_objpos_if_misc_ = NULL;
    l_track_if_ = NULL;
    laser_segmentation_if_ = NULL;
    switch_if_ = NULL;
    target_if_ = NULL;
    visdis_if_ = NULL;
    line_if_ = NULL;

    tb_connection_->set_stock_id(Gtk::Stock::CONNECT);
    tb_select_->set_sensitive(false);
    tb_lines_->set_sensitive(false);
    tb_points_->set_sensitive(false);
    tb_hull_->set_sensitive(false);
    tb_trimvals_->set_sensitive(false);
    tb_rotation_->set_sensitive(false);
    tb_legtracker_->set_sensitive(false);
    tb_stop_->set_sensitive(false);
    tb_zoom_in_->set_sensitive(false);
    tb_zoom_out_->set_sensitive(false);
  }

  
  /** Event handler for rotation button. */
  void on_rotation_toggled()
  {
    if ( tb_rotation_->get_active() ) {
      area_->set_rotation(M_PI / 2);
    } else {
      area_->set_rotation(0);
    }
  }
  

  /** Event handler for stop button */
  void on_stop_toggled()
  {
    area_->toggle_break_drawing();
  }

  /** Event handler for legtracker button */
  void on_legtracker_toggled()
  {
    if (! bb_)  return;

    if (!tb_legtracker_->get_active()) {
      bb_->close(laser_segmentation_if_);
      bb_->close(switch_if_);
      bb_->close(target_if_);

      std::list<ObjectPositionInterface*>::iterator objpos_if_itt;
      std::list<Position2DTrackInterface*>::iterator track_if_itt;
      if (l_objpos_if_persons_) {
	for( objpos_if_itt = l_objpos_if_persons_->begin(); objpos_if_itt != l_objpos_if_persons_->end(); objpos_if_itt++ ) {
	  bb_->close(*objpos_if_itt);
	}
	l_objpos_if_persons_->clear();
      }
      if (l_objpos_if_legs_) {
	for( objpos_if_itt = l_objpos_if_legs_->begin(); objpos_if_itt != l_objpos_if_legs_->end(); objpos_if_itt++ ) {
	  bb_->close(*objpos_if_itt);
	}
	l_objpos_if_legs_->clear();
      }
      if (l_objpos_if_misc_) {
	for( objpos_if_itt = l_objpos_if_misc_->begin(); objpos_if_itt != l_objpos_if_misc_->end(); objpos_if_itt++ ) {
	  bb_->close(*objpos_if_itt);
	}
	l_objpos_if_misc_->clear();
      }

      if (l_track_if_) {
	for( track_if_itt = l_track_if_->begin(); track_if_itt != l_track_if_->end(); track_if_itt++ ) {
	  bb_->close(*track_if_itt);
	}
	l_track_if_->clear();
      }
      
      laser_segmentation_if_ = NULL;
      switch_if_ = NULL;
      target_if_ = NULL;
      l_objpos_if_persons_ = NULL;
      l_objpos_if_legs_ = NULL;
      l_objpos_if_misc_ = NULL;
      l_track_if_ = NULL;

      area_->set_objpos_if(l_objpos_if_persons_,l_objpos_if_legs_,l_objpos_if_misc_,laser_segmentation_if_, l_track_if_, target_if_,switch_if_);

    } else {
      unsigned int num_opens = 3
	+ MAX_OBJECTPOSITIONINTERFACES_PERSONS
	+ MAX_OBJECTPOSITIONINTERFACES_LEGS
	+ MAX_OBJECTPOSITIONINTERFACES_MISC
	+ MAX_TRACKINTERFACES;

      float step_fraction = 1.0 / num_opens;
      unsigned int opened = 0;
      pgb_ltopen_->set_fraction(0);
      dlg_ltopen_->show();
      area_->queue_draw();

      laser_segmentation_if_ = bb_->open_for_reading<Laser720Interface>("SegmentsLaser");
      pgb_ltopen_->set_fraction(++opened * step_fraction);
      while (Gtk::Main::events_pending()) Gtk::Main::iteration();

      target_if_ = bb_->open_for_reading<ObjectPositionInterface>("legtracker Target");

      ObjectPositionInterface* new_objpos_if;
      l_objpos_if_persons_ = new std::list<ObjectPositionInterface*>();
      l_objpos_if_legs_ = new std::list<ObjectPositionInterface*>();
      l_objpos_if_misc_ = new std::list<ObjectPositionInterface*>();
      l_track_if_ = new std::list<Position2DTrackInterface*>();
      for (int i=1; i <= MAX_OBJECTPOSITIONINTERFACES_PERSONS; ++i){
	new_objpos_if= bb_->open_for_reading<ObjectPositionInterface>((std::string("legtracker CurrentLegsTracked") + StringConversions::to_string(i)).c_str());
	l_objpos_if_persons_->push_back(new_objpos_if);
	pgb_ltopen_->set_fraction(++opened * step_fraction);
	while (Gtk::Main::events_pending()) Gtk::Main::iteration();
      }
      for (int i=1; i <= MAX_OBJECTPOSITIONINTERFACES_LEGS; ++i){
	new_objpos_if= bb_->open_for_reading<ObjectPositionInterface>((std::string("legtracker Leg") + StringConversions::to_string(i)).c_str());
	l_objpos_if_legs_->push_back(new_objpos_if);
	pgb_ltopen_->set_fraction(++opened * step_fraction);
	while (Gtk::Main::events_pending()) Gtk::Main::iteration();
      }
      for (int i=1; i <= MAX_OBJECTPOSITIONINTERFACES_MISC; ++i){
	new_objpos_if= bb_->open_for_reading<ObjectPositionInterface>((std::string("legtracker Misc") + StringConversions::to_string(i)).c_str());
	l_objpos_if_misc_->push_back(new_objpos_if);
	pgb_ltopen_->set_fraction(++opened * step_fraction);
	while (Gtk::Main::events_pending()) Gtk::Main::iteration();
      }
      Position2DTrackInterface* new_track_if;
      for (int i=1; i <= MAX_TRACKINTERFACES; ++i){
	new_track_if = bb_->open_for_reading<Position2DTrackInterface>((std::string("legtracker Track") + StringConversions::to_string(i)).c_str());
	l_track_if_->push_back( new_track_if );
	pgb_ltopen_->set_fraction(++opened * step_fraction);
	while (Gtk::Main::events_pending()) Gtk::Main::iteration();
      }
      
      switch_if_ = bb_->open_for_reading<SwitchInterface>("legtracker write!");
      pgb_ltopen_->set_fraction(++opened * step_fraction);
      while (Gtk::Main::events_pending()) Gtk::Main::iteration();
      dlg_ltopen_->hide();
      area_->set_objpos_if(l_objpos_if_persons_, l_objpos_if_legs_,
			    l_objpos_if_misc_,laser_segmentation_if_,
			    l_track_if_, target_if_,switch_if_);
      area_->queue_draw();
    }
  }


  /** Event handler for trim button. */
  void on_trimvals_toggled()
  {
    if ( tb_trimvals_->get_active() ) {
      area_->set_resolution(3);
    } else {
      area_->set_resolution(1);
    }
  }

  /** Event handler for exit button. */
  void on_exit_clicked()
  {
    Gtk::Main::quit();
  }

 private:
  BlackBoard                        *bb_;
  InterfaceDispatcherPairList        laser_ifs_;
  Laser720Interface                 *laser_segmentation_if_;
  SwitchInterface                   *switch_if_;
  ObjectPositionInterface           *target_if_;
  
  std::list<ObjectPositionInterface*>* l_objpos_if_persons_;
  std::list<ObjectPositionInterface*>* l_objpos_if_legs_;
  std::list<ObjectPositionInterface*>* l_objpos_if_misc_;
  std::list<Position2DTrackInterface*>* l_track_if_;

  ObjectPositionInterface           *line_if_;
  VisualDisplay2DInterface          *visdis_if_;

  LaserDrawingArea                  *area_;
  AllemaniACsAtHomeCairoRobotDrawer  athome_drawer_;
  ConnectionDispatcher               connection_dispatcher_;

  Gtk::ToolButton                    *tb_connection_;
  Gtk::RadioToolButton               *tb_lines_;
  Gtk::RadioToolButton               *tb_points_;
  Gtk::RadioToolButton               *tb_hull_;
  Gtk::ToggleToolButton              *tb_trimvals_;
  Gtk::ToggleToolButton              *tb_rotation_;
  Gtk::ToggleToolButton              *tb_legtracker_;
  Gtk::ToggleToolButton              *tb_stop_;
  Gtk::ToolButton                    *tb_zoom_in_;
  Gtk::ToolButton                    *tb_zoom_out_;
  Gtk::ToolButton                    *tb_exit_;
  Gtk::ToolButton                    *tb_select_;

  Gtk::Dialog                        *dlg_ltopen_;
  Gtk::ProgressBar                   *pgb_ltopen_;

  TypeIdPairList                      laser_if_names_;
};

int
main(int argc, char** argv)
{
   Gtk::Main kit(argc, argv);
#ifdef HAVE_GCONFMM
  Gnome::Conf::init();
#endif
   
   Glib::RefPtr<Gtk::Builder> builder;
   builder = Gtk::Builder::create_from_file(RESDIR"/guis/lasergui/lasergui.ui");

   LaserGuiGtkWindow *window = NULL;
   builder->get_widget_derived("wnd_lasergui", window);

   Gtk::Main::run(*window);

   return 0;
}
