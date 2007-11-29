/***************************************************************************
 *  navigator_gui.h - Navigator GUI
 *
 *  Generated: Fri Jun 01 11:41:34 2007
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __TOOLS_NAVIGATOR_GUI_NAVIGATOR_GUI_H_
#define __TOOLS_NAVIGATOR_GUI_NAVIGATOR_GUI_H_

#include <plugins/navigator/libnavi/npoint.h>

#include <libs/netcomm/fawkes/client_handler.h>
#include <libs/core/utils/lock_list.h>

#include <gtkmm/drawingarea.h>
#include <gtkmm/radiobutton.h>

#include <vector>

class FawkesNetworkClient;
class FawkesNetworkMessage;
class NLine;
class Mutex;
class Obstacle;

namespace Gtk
  {
  class Button;
  class HScale;
  class Adjustment;
  class SpinButton;
  class Label;
  class RadioButtonGroup;
  class RadioButton;
  class CheckButton;
  class EntryCompletion;
  class Window;
  class VBox;
  class HBox;
  class ButtonBox;
  class Frame;
  class Alignment;
}

class NavigatorGUI : public  Gtk::DrawingArea, public FawkesNetworkClientHandler
  {
  public:

    NavigatorGUI(const char *host_name);
    virtual ~NavigatorGUI();

  private:

    Gtk::Window* win;

    Gtk::Button* control_button;
    Gtk::Button* stop_button;
    Gtk::Button* send_button;
    Gtk::Button* reset_odometry_button;
    Gtk::HScale* zooming_HScale;

    Gtk::SpinButton* right_rpm_entry;
    Gtk::SpinButton* left_rpm_entry;
    Gtk::SpinButton* center_rpm_entry;
    Gtk::SpinButton* xv_entry;
    Gtk::SpinButton* yv_entry;
    Gtk::SpinButton* rotation_entry;
    Gtk::SpinButton* angular_velocity_entry;
    Gtk::SpinButton* navigator_velocity_entry;

    Gtk::Label* right_rpm_label;
    Gtk::Label* left_rpm_label;
    Gtk::Label* center_rpm_label;
    Gtk::Label* xv_label;
    Gtk::Label* yv_label;
    Gtk::Label* rotation_label;
    Gtk::Label* angular_velocity_label;
    Gtk::Label* navigator_velocity_label;

    Gtk::RadioButton* navigator_control_radio;
    Gtk::RadioButton* motor_control_radio;
    Gtk::RadioButton* behold_radio;

    Gtk::CheckButton* obstacle_check;

    Gtk::RadioButton* rpm_radio;
    Gtk::RadioButton* trans_rot_radio;
    Gtk::RadioButton* trans_radio;
    Gtk::RadioButton* rot_radio;
    Gtk::RadioButton* orbit_radio;
    Gtk::RadioButton* navigator_radio;

    Gtk::Adjustment *zooming_adjustment;

    Gtk::VBox* m_VBox_Main;
    Gtk::HBox* m_HBox_Left;
    Gtk::ButtonBox* bbox_top;
    Gtk::VBox* bbox_left;
    Gtk::VBox* control_VBox;


    Gtk::RadioButton::Group control_group;

    Gtk::RadioButton::Group drive_mode_group;
    Gtk::Frame* rpm_frame;
    Gtk::Frame* velocity_frame;
    Gtk::Frame* orbit_frame;
    Gtk::Frame* navigator_frame;

    Gtk::ButtonBox* rpm_entry_box;
    Gtk::ButtonBox* velocity_entry_box;

    Gtk::ButtonBox* rpm_label_box1;
    Gtk::ButtonBox* rpm_label_box2;
    Gtk::ButtonBox* rpm_label_box3;
    Gtk::ButtonBox* velocity_label_box1;
    Gtk::ButtonBox* velocity_label_box2;
    Gtk::ButtonBox* velocity_label_box3;
    Gtk::ButtonBox* orbit_label_box;
    Gtk::ButtonBox* navigator_label_box;


    Gtk::Alignment* left_rpm_alignment;
    Gtk::Alignment* center_rpm_alignment;
    Gtk::Alignment* right_rpm_alignment;

    Gtk::Alignment* x_velocity_alignment;
    Gtk::Alignment* y_velocity_alignment;
    Gtk::Alignment* rotation_velocity_alignment;

    Gtk::Alignment* orbit_alignment;
    Gtk::Alignment* navigator_alignment;


    double window_width;
    double window_height;
    double scanning_area_width;
    double scanning_area_height;

    double zoom_factor;

    double point_radius;
    double robot_radius;

    NPoint ball_point;
    NPoint odometry_point;
    NPoint target_point;
    NPoint cursor_point;
    double odometry_orientation;
    double robot_orientation;
    double odometry_direction;

    LockList<NPoint*> points;
    LockList<NLine*> lines;
    LockList<NPoint*> path_points;
    LockList<Obstacle*> obstacle_points;

    Mutex *odometry_orientation_mutex;
    Mutex *ball_point_mutex;
    Mutex *odometry_point_mutex;
    Mutex *target_point_mutex;
    Mutex *cursor_point_mutex;
    bool cursor_over_area;

    double velocity;


    FawkesNetworkClient *net_client;
    void deregistered() throw();
    void inbound_received(FawkesNetworkMessage *msg) throw();
    void process_navigator_message(FawkesNetworkMessage *msg) throw();
    void process_pluginmanager_message(FawkesNetworkMessage *msg) throw();
    bool navigator_loaded;
    
    void connection_established() throw();
    void connection_died() throw();

    bool on_idle();
    bool on_expose_event(GdkEventExpose* event);
    bool on_button_press_event(GdkEventButton* event);
    bool on_button_release_event(GdkEventButton* event);
    bool on_motion_notify_event(GdkEventMotion* event);
    bool on_leave_notify_event(GdkEventCrossing* event);
    bool on_enter_notify_event(GdkEventCrossing* event);
    void on_navigator_control_radio_clicked();
    void on_motor_control_radio_clicked();
    void on_behold_radio_clicked();
    void on_stop_button_clicked();
    void on_send_button_clicked();
    void on_reset_odometry_button_clicked();
    void on_zooming_value_changed();
    void send_stop();
    bool navigator_control;
    bool motor_control;
  };

#endif /*NAVIGATOR_GUI_H_*/
