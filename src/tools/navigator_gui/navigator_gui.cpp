/***************************************************************************
 *  navigator_gui.cpp - Navigator GUI
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

#include <tools/navigator_gui/navigator_gui.h>

#include <plugins/navigator/libnavi/navigator_messages.h>
#include <plugins/navigator/libnavi/nline.h>
#include <plugins/navigator/libnavi/npoint.h>
#include <plugins/navigator/libnavi/obstacle.h>

#include <mainapp/plugin_messages.h>
#include <mainapp/plugin_list_message.h>
#include <utils/system/argparser.h>
#include <netcomm/socket/socket.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/message.h>
#include <utils/math/angle.h>
#include <utils/math/binomial_coefficient.h>

#include <iostream>
#include <unistd.h>
#include <fstream>

#include <gtkmm/main.h>
#include <gtkmm/window.h>
#include <gtkmm/button.h>

#include <cairomm/context.h>
#include <gdkmm/drawable.h>
#include <gtkmm.h>

/** @class NavigatorGUI <tools/navigator_gui/navigator_gui.h>
 *   A GUI to control, simulate and monitor the behavior of the navigator.
 *   
 *   Usage:
 *   >navigator_gui
 *   Opens a connection to the locally running navigator plugin.
 *   >navigator_gui -r 172.16.35.101
 *   Opens a connection to a running navigator plugin 
 *   on the host with the IP address 172.16.35.101.
 *   
 *   @author Martin Liebenberg
 */

/** Constructor.
 * @param host_name the host name of the host which to connect to
 */
NavigatorGUI::NavigatorGUI(const char *host_name)
{
  this->host_name = host_name;
  zoom_factor = 100;

  win = new Gtk::Window();

  win->set_title("Navigator GUI");
  win->resize(990, 700);


  m_VBox_Main = new Gtk::VBox();
  m_HBox_Left = new Gtk::HBox();

  right_rpm_entry = new Gtk::SpinButton(1.0, 2);
  right_rpm_entry->set_range(-5000.0, 5000.0);
  right_rpm_entry->set_increments(1.0, 10.0);
  right_rpm_entry->set_width_chars(5);
  left_rpm_entry = new Gtk::SpinButton(1.0, 2);
  left_rpm_entry->set_range(-5000.0, 5000.0);
  left_rpm_entry->set_increments(1.0, 10.0);
  left_rpm_entry->set_width_chars(5);
  center_rpm_entry = new Gtk::SpinButton(1.0, 2);
  center_rpm_entry->set_range(-5000.0, 5000.0);
  center_rpm_entry->set_increments(1.0, 10.0);
  center_rpm_entry->set_width_chars(5);
  xv_entry = new Gtk::SpinButton(0.005, 2);
  xv_entry->set_range(-5.0, 5.0);
  xv_entry->set_increments(0.01, 0.1);
  xv_entry->set_width_chars(5);
  yv_entry = new Gtk::SpinButton(0.005, 2);
  yv_entry->set_range(-5.0, 5.0);
  yv_entry->set_increments(0.01, 0.1);
  yv_entry->set_width_chars(5);
  rotation_entry = new Gtk::SpinButton(0.005, 2);
  rotation_entry->set_range(-6.3, 6.3);
  rotation_entry->set_increments(0.01, 0.1);
  rotation_entry->set_width_chars(5);
  angular_velocity_entry = new Gtk::SpinButton(0.005, 2);
  angular_velocity_entry->set_range(-6.3, 6.3);
  angular_velocity_entry->set_increments(0.01, 0.1);
  navigator_velocity_entry = new Gtk::SpinButton(0.005, 2);
  navigator_velocity_entry->set_range(0, 5.0);
  navigator_velocity_entry->set_increments(0.01, 0.1);

  navigator_control_radio = new Gtk::RadioButton("Navigator Control");
  motor_control_radio = new Gtk::RadioButton("Motor Control");
  behold_radio = new Gtk::RadioButton("Behold");

  rpm_radio = new Gtk::RadioButton("RPM");
  trans_rot_radio = new Gtk::RadioButton("Trans-Rot");
  trans_radio = new Gtk::RadioButton("Trans");
  rot_radio = new Gtk::RadioButton("Rot");
  orbit_radio = new Gtk::RadioButton("Orbit");
  line_trans_rot_radio = new Gtk::RadioButton("Line-Trans-Rot");
  navigator_radio = new Gtk::RadioButton("Navigator");

  obstacle_check = new Gtk::CheckButton("add Obstacles");
  orientation_check = new Gtk::CheckButton("orientate");
  debug_check = new Gtk::CheckButton("Debugging");

  right_rpm_label = new Gtk::Label("Right");
  left_rpm_label = new Gtk::Label("Left");
  center_rpm_label = new Gtk::Label("Rear");
  xv_label = new Gtk::Label("X");
  yv_label = new Gtk::Label("Y");
  angular_velocity_label = new Gtk::Label("Angular Velocity [rad/s]\nSet the orbit center ->>");
  navigator_velocity_label = new Gtk::Label("Maximum Velocity [m/s]");
  zooming_label = new Gtk::Label("Zoom");

  control_button = new Gtk::Button("Start Control");
  stop_button = new Gtk::Button("STOP");
  stop_button->modify_bg(Gtk::STATE_NORMAL, Gdk::Color("red"));
  stop_button->modify_bg(Gtk::STATE_ACTIVE , Gdk::Color("red"));
  stop_button->modify_bg(Gtk::STATE_PRELIGHT , Gdk::Color("red"));
  stop_button->modify_bg(Gtk::STATE_SELECTED , Gdk::Color("red"));
  stop_button->modify_bg(Gtk::STATE_INSENSITIVE , Gdk::Color("red"));
  send_button = new Gtk::Button("Send Command");
  reset_odometry_button = new Gtk::Button("Reset Odometry");
  navigator_control = false;
  motor_control = false;
  cursor_over_area = false;
  navigator_loaded = true;
  connected = false;

  zooming_adjustment = new Gtk::Adjustment(0.0, -99.9, 100.0);
  zooming_HScale = new Gtk::HScale(*zooming_adjustment);
  zooming_HScale->set_size_request (200, -1);

  bbox_top = 0;
  bbox_left = new Gtk::VBox();
  control_VBox = new Gtk::VBox();


  control_group = navigator_control_radio->get_group();
  motor_control_radio->set_group(control_group);
  behold_radio->set_group(control_group);

  control_VBox->add(*navigator_control_radio);
  control_VBox->add(*motor_control_radio);
  control_VBox->add(*behold_radio);
  behold_radio->set_active();

  drive_mode_group = rpm_radio->get_group();
  trans_rot_radio->set_group(drive_mode_group);
  trans_radio->set_group(drive_mode_group);
  rot_radio->set_group(drive_mode_group);
  orbit_radio->set_group(drive_mode_group);
  line_trans_rot_radio->set_group(drive_mode_group);
  navigator_radio->set_group(drive_mode_group);

  trans_rot_radio->set_active();

  rpm_frame = new Gtk::Frame("RPMs");
  translation_frame = new Gtk::Frame("Translation [m/s]");
  rotation_frame = new Gtk::Frame("Rotation [rad/s]");
  orbit_frame = new Gtk::Frame("Orbit");
  navigator_frame = new Gtk::Frame("Navigator");
  status_frame = new Gtk::Frame();


  rpm_entry_box = Gtk::manage( new Gtk::HButtonBox() );
  trans_rot_entry_box = new Gtk::HBox();

  rpm_label_box1 = Gtk::manage( new Gtk::VButtonBox() );
  rpm_label_box2 = Gtk::manage( new Gtk::VButtonBox() );
  rpm_label_box3 = Gtk::manage( new Gtk::VButtonBox() );
  translation_x_label = Gtk::manage( new Gtk::VButtonBox() );
  translation_y_label = Gtk::manage( new Gtk::VButtonBox() );
  orbit_label_box = Gtk::manage( new Gtk::VButtonBox() );
  navigator_label_box = Gtk::manage( new Gtk::VButtonBox() );
  zoom_debug_box = Gtk::manage( new Gtk::VButtonBox() );
  
  zoom_debug_box->add(*debug_check);
  zoom_debug_box->add(*zooming_HScale);

  bbox_top = Gtk::manage( new Gtk::HButtonBox(Gtk::BUTTONBOX_SPREAD, 30) );
  bbox_top->set_border_width(5);
  bbox_top->add(*zoom_debug_box);
  bbox_top->add(*stop_button);
  bbox_top->add(*control_VBox);
  bbox_top->add(*reset_odometry_button);

  left_rpm_alignment = new Gtk::Alignment(Gtk::ALIGN_CENTER, Gtk::ALIGN_CENTER, 0.0, 0.0);
  center_rpm_alignment = new Gtk::Alignment(Gtk::ALIGN_CENTER, Gtk::ALIGN_CENTER, 0.0, 0.0);
  right_rpm_alignment = new Gtk::Alignment(Gtk::ALIGN_CENTER, Gtk::ALIGN_CENTER, 0.0, 0.0);

  x_translation_alignment = new Gtk::Alignment(Gtk::ALIGN_CENTER, Gtk::ALIGN_CENTER, 0.0, 0.0);
  y_translation_alignment = new Gtk::Alignment(Gtk::ALIGN_CENTER, Gtk::ALIGN_CENTER, 0.0, 0.0);
  rotation_alignment = new Gtk::Alignment(Gtk::ALIGN_CENTER, Gtk::ALIGN_BOTTOM, 0.0, 0.0);
  rotation_frame_alignment = new Gtk::Alignment(Gtk::ALIGN_CENTER, Gtk::ALIGN_CENTER, 0.0, 1.0);

  orbit_alignment = new Gtk::Alignment(Gtk::ALIGN_CENTER, Gtk::ALIGN_CENTER, 0.0, 0.0);
  navigator_alignment = new Gtk::Alignment(Gtk::ALIGN_CENTER, Gtk::ALIGN_CENTER, 0.0, 0.0);
  left_rpm_alignment->add(*left_rpm_label);
  center_rpm_alignment->add(*center_rpm_label);
  right_rpm_alignment->add(*right_rpm_label);

  x_translation_alignment->add(*xv_label);
  y_translation_alignment->add(*yv_label);

  orbit_alignment->add(*angular_velocity_label);
  navigator_alignment->add(*navigator_velocity_label);

  rpm_label_box1->add(*left_rpm_alignment);
  rpm_label_box1->add(*left_rpm_entry);
  rpm_entry_box->add(*rpm_label_box1);

  rpm_label_box2->add(*center_rpm_alignment);
  rpm_label_box2->add(*center_rpm_entry);
  rpm_entry_box->add(*rpm_label_box2);

  rpm_label_box3->add(*right_rpm_alignment);
  rpm_label_box3->add(*right_rpm_entry);
  rpm_entry_box->add(*rpm_label_box3);

  translation_HBox = new Gtk::HBox(true, 5);
  translation_x_label->add(*x_translation_alignment);
  translation_x_label->add(*xv_entry);
  translation_HBox->add(*translation_x_label);

  translation_y_label->add(*y_translation_alignment);
  translation_y_label->add(*yv_entry);
  translation_HBox->add(*translation_y_label);
  translation_frame->add(*translation_HBox);
  trans_rot_entry_box->pack_start(*translation_frame, Gtk::PACK_SHRINK , 4);

  rotation_alignment->add(*rotation_entry);
  rotation_frame->add(*rotation_alignment);
  rotation_frame_alignment->add(*rotation_frame);
  trans_rot_entry_box->add(*rotation_frame_alignment);

  orbit_label_box->add(*orbit_alignment);
  orbit_label_box->add(*angular_velocity_entry);

  navigator_label_box->add(*navigator_alignment);
  navigator_label_box->add(*navigator_velocity_entry);
  navigator_label_box->add(*obstacle_check);
  navigator_label_box->add(*orientation_check);

  rpm_frame->add(*rpm_entry_box);
  orbit_frame->add(*orbit_label_box);
  navigator_frame->add(*navigator_label_box);

  statusbar = new Gtk::Statusbar();
  statusbar->push("Not connected.", 0);
  statusbar->set_has_resize_grip(false);
  status_frame->add(*statusbar);

  bbox_left->pack_start(*rpm_frame, Gtk::PACK_SHRINK, 4);
  bbox_left->pack_start(*trans_rot_entry_box, Gtk::PACK_SHRINK, 4);
  bbox_left->pack_start(*orbit_frame, Gtk::PACK_SHRINK, 4);
  bbox_left->pack_start(*navigator_frame, Gtk::PACK_SHRINK, 4);
  bbox_left->pack_start(*rpm_radio, Gtk::PACK_SHRINK, 4);
  bbox_left->pack_start(*trans_rot_radio, Gtk::PACK_SHRINK, 4);
  bbox_left->pack_start(*trans_radio, Gtk::PACK_SHRINK, 4);
  bbox_left->pack_start(*rot_radio, Gtk::PACK_SHRINK, 4);
  bbox_left->pack_start(*orbit_radio, Gtk::PACK_SHRINK, 4);
  bbox_left->pack_start(*line_trans_rot_radio, Gtk::PACK_SHRINK, 4);
  bbox_left->pack_start(*send_button, Gtk::PACK_SHRINK, 10);

  m_VBox_Main->pack_start(*bbox_top, Gtk::PACK_SHRINK, 4);
  m_VBox_Main->pack_start(*m_HBox_Left, Gtk::PACK_EXPAND_WIDGET);
  m_VBox_Main->pack_start(*status_frame, Gtk::PACK_SHRINK);
  m_HBox_Left->pack_start(*bbox_left, Gtk::PACK_SHRINK, 5);
  m_HBox_Left->pack_start(*this, Gtk::PACK_EXPAND_WIDGET);
  win->add(*m_VBox_Main);



  zooming_adjustment->signal_value_changed().connect(sigc::mem_fun(*this,
      &NavigatorGUI::on_zooming_value_changed));
  navigator_control_radio->signal_pressed().connect( sigc::mem_fun(*this,
      &NavigatorGUI::on_navigator_control_radio_clicked) );
  motor_control_radio->signal_pressed().connect( sigc::mem_fun(*this,
      &NavigatorGUI::on_motor_control_radio_clicked) );
  behold_radio->signal_pressed().connect( sigc::mem_fun(*this,
                                          &NavigatorGUI::on_behold_radio_clicked) );
  stop_button->signal_clicked().connect( sigc::mem_fun(*this,
                                         &NavigatorGUI::on_stop_button_clicked) );
  send_button->signal_clicked().connect( sigc::mem_fun(*this,
                                         &NavigatorGUI::on_send_button_clicked) );
  reset_odometry_button->signal_clicked().connect( sigc::mem_fun(*this,
      &NavigatorGUI::on_reset_odometry_button_clicked) );

  odometry_orientation_mutex = new Mutex();
  odometry_point_mutex = new Mutex();
  cursor_point_mutex = new Mutex();
  ball_point_mutex = new Mutex();
  mouse_point_mutex = new Mutex();

  point_radius = 5.5;
  robot_radius = 0.25;

  //the points in screen coordinates
  cursor_point.x = 0.;
  cursor_point.y = 0.;
  odometry_point.x = 0.;
  odometry_point.y = 0.;
  ball_point.x = 1000000.;
  ball_point.y = 1000000.;
  mouse_point.x = 0.;
  mouse_point.y = 0.;

  odometry_direction = 0.;

  //currently not in use
  odometry_orientation = 0.;
  velocity = 0;

  orientating = false;
  orientation = 0;

  set_events(Gdk::BUTTON_PRESS_MASK);
  add_events(Gdk::BUTTON_RELEASE_MASK);
  add_events(Gdk::POINTER_MOTION_MASK);
  add_events(Gdk::LEAVE_NOTIFY_MASK);
  add_events(Gdk::ENTER_NOTIFY_MASK);

  Glib::signal_idle().connect( sigc::mem_fun(*this, &NavigatorGUI::on_idle) );

  net_client = NULL;
  connect();

  win->show_all_children();

  Gtk::Main::run(*win);
}

/** Decunstructor. */
NavigatorGUI::~NavigatorGUI()
{
  send_stop();

  net_client->deregister_handler(FAWKES_CID_PLUGINMANAGER);
  net_client->deregister_handler(FAWKES_CID_NAVIGATOR_PLUGIN);
  net_client->disconnect();
  delete net_client;

  //delete lines
  lines.lock();
  for(LockList<NLine*>::iterator iterator = lines.begin();
      iterator != lines.end(); iterator++)
    {
      delete *iterator;
    }
  lines.clear();
  lines.unlock();


  //delete points
  points.lock();
  for(LockList<NPoint*>::iterator iterator = points.begin();
      iterator != points.end(); iterator++)
    {
      delete *iterator;
    }
  points.clear();
  points.unlock();

  //delete path_points
  path_points.lock();
  for(LockList<NPoint*>::iterator iterator = path_points.begin();
      iterator != path_points.end(); iterator++)
    {
      delete *iterator;
    }
  path_points.clear();
  path_points.unlock();

  delete mouse_point_mutex;
  delete odometry_orientation_mutex;
  delete odometry_point_mutex;
  delete cursor_point_mutex;
  delete ball_point_mutex;
}

void NavigatorGUI::connect()
{
  if(connected)
    {
      connected = false;
      net_client->deregister_handler(FAWKES_CID_PLUGINMANAGER);
      net_client->deregister_handler(FAWKES_CID_NAVIGATOR_PLUGIN);
      net_client->cancel();
      net_client->join();
    }

  if(net_client != NULL)
    {
      delete net_client;
    }
  net_client = new FawkesNetworkClient(host_name, 1910);

  net_client->register_handler(this, FAWKES_CID_NAVIGATOR_PLUGIN);
  net_client->register_handler(this, FAWKES_CID_PLUGINMANAGER);

  try
    {
      net_client->connect();
      connection_is_dead = false;

      net_client->start();

      FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
                                   MSG_PLUGIN_SUBSCRIBE_WATCH);
      net_client->enqueue(msg1);
      msg1->unref();
      FawkesNetworkMessage *msg2 = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
                                   MSG_PLUGIN_LIST_LOADED);
      net_client->enqueue(msg2);
      msg2->unref();

      char str[100];
      strcpy (str,"Connected to ");
      strcat (str, host_name);
      strcat (str, ", but the navigator plugin is not loaded.");
      statusbar->push(str, 1);
    }
  catch (SocketException &e)
    {
      connection_is_dead = true;
      std::cerr << "------------------------------------------------------------" <<  std::endl;
      std::cerr << "There is maybe no fawkes running at " << host_name << "!" <<  std::endl;
      std::cerr << "------------------------------------------------------------" <<  std::endl;
    }
}


void
NavigatorGUI::deregistered() throw()
{
  printf("Got deregistered\n");
}

void
NavigatorGUI::connection_established() throw()
{
  printf("Connection established\n");
  connected = true;
}


void
NavigatorGUI::connection_died() throw()
{
  printf("Connection died\n");
  statusbar->pop(1);

  printf("Connection died2\n");
  //net_client->deregister_handler(FAWKES_CID_PLUGINMANAGER);
  // net_client->deregister_handler(FAWKES_CID_NAVIGATOR_PLUGIN);
  /*
  printf("Connection cancel\n");
  net_client->cancel();
  printf("Connection join\n");
  net_client->join();*/
  printf("Connection is dead\n");

  connection_is_dead = true;
}

/**
 * Delets all displayable stuff and resets some values.
 */
void
NavigatorGUI::reset_gui()
{
  odometry_direction = 0.;
  odometry_orientation = 0.;
  odometry_point.x = 0.;
  odometry_point.y = 0.;
  mouse_point.x = 0.;
  mouse_point.y = 0.;
  ball_point.x = 1000000.;
  ball_point.y = 1000000.;
  navigator_control = false;
  motor_control = false;
  navigator_loaded = false;

  //delete lines
  lines.lock();
  for(LockList<NLine*>::iterator iterator = lines.begin();
      iterator != lines.end(); iterator++)
    {
      delete *iterator;
    }
  lines.clear();
  lines.unlock();

  //delete points
  points.lock();
  for(LockList<NPoint*>::iterator iterator = points.begin();
      iterator != points.end(); iterator++)
    {
      delete *iterator;
    }
  points.clear();
  points.unlock();

  //delete path_points
  path_points.lock();
  for(LockList<NPoint*>::iterator iterator = path_points.begin();
      iterator != path_points.end(); iterator++)
    {
      delete *iterator;
    }
  path_points.clear();
  path_points.unlock();

  //delete obstacle_points
  obstacle_points.lock();
  for(LockList<Obstacle*>::iterator iterator = obstacle_points.begin();
      iterator != obstacle_points.end(); iterator++)
    {
      delete *iterator;
    }
  obstacle_points.clear();
  obstacle_points.unlock();
}

/** Inbound mesage received.
 * @param m message
 */
void
NavigatorGUI::inbound_received(FawkesNetworkMessage *msg) throw()
{
  if(msg->cid() == FAWKES_CID_NAVIGATOR_PLUGIN)
    {
      process_navigator_message(msg);
    }
  else if(msg->cid() == FAWKES_CID_PLUGINMANAGER)
    {
      process_pluginmanager_message(msg);
    }
}

/** Process messages from the navigator.
 * @param m message
 */
void NavigatorGUI::process_navigator_message(FawkesNetworkMessage *msg) throw()
{
  if(msg->msgid() == NAVIGATOR_MSGTYPE_CONTROL_SUBERR)
    {
      std::cerr << "Error: Subscribing failed.\nMaybe there is already a controlling tool subscribed." << std::endl;
      navigator_control = false;
      motor_control = false;
      behold_radio->set_active();
      control_button->set_label("Start Control");
    }
  else if (NAVIGATOR_MSGTYPE_SURFACE == msg->msgid() )
    {
      NavigatorSurfaceMessage *surface_msg = msg->msgc<NavigatorSurfaceMessage>();

      //delete lines
      lines.lock();
      for(LockList<NLine*>::iterator iterator = lines.begin();
          iterator != lines.end(); iterator++)
        {
          delete *iterator;
        }
      lines.clear();

      //add lines
      while (surface_msg->has_next())
        {
          NavigatorSurfaceMessage::nline_t *l = surface_msg->next();
          NPoint *p1 = 0;
          NPoint *p2 = 0;

          if ( l->width1 != 0. )
            {
              p1 = new Obstacle(l->width1, -l->y1, -l->x1, 0);
            }
          else
            {
              p1 = new NPoint(-l->y1, -l->x1);
            }

          if ( l->width2 != 0. )
            {
              p2 = new Obstacle(l->width2, -l->y2, -l->x2, 0);
            }
          else
            {
              p2 = new NPoint(-l->y2, -l->x2);
            }
          lines.push_back(new NLine(p1, p2));
        }
      lines.unlock();

      delete surface_msg;
    }
  else if (NAVIGATOR_MSGTYPE_PATH == msg->msgid() )
    {
      NavigatorPathListMessage *path_msg = msg->msgc<NavigatorPathListMessage>();

      //delete path_points
      path_points.lock();
      for(LockList<NPoint*>::iterator iterator = path_points.begin();
          iterator != path_points.end(); iterator++)
        {
          delete *iterator;
        }
      path_points.clear();

      //add path_points
      while (path_msg->has_next())
        {
          NavigatorPathListMessage::npoint_t *p = path_msg->next();
          // std::cout << "p " << p->x << ", " << p->y << std::endl;
          path_points.push_back(new NPoint(-p->y, -p->x));
        }
      path_points.unlock();

      delete path_msg;
    }
  else if (NAVIGATOR_MSGTYPE_ODOMETRY == msg->msgid())
    {
      navigator_odometry_message_t *odometry_msg =  (navigator_odometry_message_t *)msg->payload();

      odometry_point_mutex->lock();
      odometry_point.x = -odometry_msg->position_y;
      odometry_point.y = -odometry_msg->position_x;
      if(odometry_msg->velocity_y == 0. && odometry_msg->velocity_x == 0.)
        {
          odometry_direction = 0.;
        }
      else
        {
          odometry_direction = -atan2(odometry_msg->velocity_y, odometry_msg->velocity_x);
        }
      odometry_point_mutex->unlock();
    }
  else if (NAVIGATOR_MSGTYPE_BALL == msg->msgid())
    {
      navigator_ball_message_t *ball_msg =  (navigator_ball_message_t *)msg->payload();
      ball_point_mutex->lock();
      ball_point.x = -ball_msg->y * zoom_factor;
      ball_point.y = -ball_msg->x * zoom_factor;
      ball_point_mutex->unlock();
    }
}

/** Prepares the GUI for contact with the navigator.
 * Sets the statur bar and send a subscribe message to the navigator for receiving gui stuff.
 */
void NavigatorGUI::prepare_navigator_contact()
{
  std::cerr << "Navigator plugin is loaded." << std::endl;
  navigator_loaded = true;
  char * str = new char[100];
  strcpy(str, "Connected to ");
  strcat(str, host_name);
  strcat(str, " and NavigatorPlugin is loaded.");
  statusbar->push(str, 1);
  navigator_subscribe_message_t *sub_msg = (navigator_subscribe_message_t *)calloc(1, sizeof(navigator_subscribe_message_t));
  sub_msg->sub_type_points_and_lines = 1;
  sub_msg->sub_type_odometry = 1;
  sub_msg->sub_type_ball = 1;
  FawkesNetworkMessage *smsg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN,
                               NAVIGATOR_MSGTYPE_SUBSCRIBE, sub_msg, sizeof(navigator_subscribe_message_t));
  net_client->enqueue(smsg);
  smsg->unref();
}


/** Process messages from the plugin manager.
 * @param m message
 */
void NavigatorGUI::process_pluginmanager_message(FawkesNetworkMessage *msg) throw()
{
  if(msg->msgid() == MSG_PLUGIN_UNLOADED)
    {
      std::cerr << "Received unload message." << std::endl;
      plugin_unloaded_msg_t *u = (plugin_unloaded_msg_t *)msg->payload();

      if(strncmp("navigator", u->name, PLUGIN_MSG_NAME_LENGTH) == 0)
        {
          std::cerr << "Plugin " << u->name << " is unloaded." << std::endl;
          statusbar->pop(1);
          reset_gui();
        }
    }
  else if(msg->msgid() == MSG_PLUGIN_LOADED)
    {
      std::cerr << "Received load message." << std::endl;
      plugin_loaded_msg_t *lm= (plugin_loaded_msg_t *)msg->payload();
      if ( strncmp("navigator", lm->name, PLUGIN_MSG_NAME_LENGTH) == 0 )
        {
          prepare_navigator_contact();
        }
    }
  else if (msg->msgid() == MSG_PLUGIN_LOADED_LIST )
    {
      std::cerr << "Received plugin list." << std::endl;
      PluginListMessage *plm = msg->msgc<PluginListMessage>();
      if ( plm->has_next() )
        {
          while ( plm->has_next() )
            {
              char *p = plm->next();
              if(strcmp(p, "navigator") == 0)
                {
                  prepare_navigator_contact();
                }
              free(p);
            }
        }
      else
        {
          navigator_loaded = false;
          std::cerr << "Navigator plugin is not loaded." << std::endl;
        }
      delete plm;
    }
}

bool NavigatorGUI::on_idle()
{
  if(motor_control)
    {
      if(rpm_radio->get_active())
        {
          translation_frame->set_sensitive(false);
          rotation_frame->set_sensitive(false);
          rpm_frame->set_sensitive(true);
          orbit_frame->set_sensitive(false);
          navigator_frame->set_sensitive(false);
          send_button->set_sensitive(true);
          mouse_point.x = 0;
          mouse_point.y = 0;
        }
      else if(trans_rot_radio->get_active())
        {
          translation_frame->set_sensitive(true);
          rotation_frame->set_sensitive(true);
          rpm_frame->set_sensitive(false);
          orbit_frame->set_sensitive(false);
          navigator_frame->set_sensitive(false);
          send_button->set_sensitive(true);
          mouse_point.x = 0;
          mouse_point.y = 0;
        }
      else if(trans_radio->get_active())
        {
          translation_frame->set_sensitive(true);
          rotation_frame->set_sensitive(false);
          rpm_frame->set_sensitive(false);
          orbit_frame->set_sensitive(false);
          navigator_frame->set_sensitive(false);
          send_button->set_sensitive(true);
          mouse_point.x = 0;
          mouse_point.y = 0;
        }
      else if(rot_radio->get_active())
        {
          translation_frame->set_sensitive(false);
          rotation_frame->set_sensitive(true);
          rpm_frame->set_sensitive(false);
          orbit_frame->set_sensitive(false);
          navigator_frame->set_sensitive(false);
          send_button->set_sensitive(true);
          mouse_point.x = 0;
          mouse_point.y = 0;
        }
      else if(orbit_radio->get_active())
        {
          translation_frame->set_sensitive(false);
          rotation_frame->set_sensitive(false);
          rpm_frame->set_sensitive(false);
          orbit_frame->set_sensitive(true);
          navigator_frame->set_sensitive(false);
          send_button->set_sensitive(true);
        }
      else if(line_trans_rot_radio->get_active())
        {
          translation_frame->set_sensitive(true);
          rotation_frame->set_sensitive(true);
          rpm_frame->set_sensitive(false);
          orbit_frame->set_sensitive(false);
          navigator_frame->set_sensitive(false);
          send_button->set_sensitive(true);
          mouse_point.x = 0;
          mouse_point.y = 0;
        }
    }

  if(navigator_control_radio->get_active())
    {
      translation_frame->set_sensitive(false);
      rotation_frame->set_sensitive(false);
      rpm_frame->set_sensitive(false);
      orbit_frame->set_sensitive(false);
      navigator_frame->set_sensitive(true);
      send_button->set_sensitive(true);
      stop_button->set_sensitive(true);

      rpm_radio->set_sensitive(false);
      trans_rot_radio->set_sensitive(false);
      trans_radio->set_sensitive(false);
      rot_radio->set_sensitive(false);
      orbit_radio->set_sensitive(false);
      line_trans_rot_radio->set_sensitive(false);
      navigator_radio->set_sensitive(false);
    }
  else if(motor_control_radio->get_active())
    {
      send_button->set_sensitive(true);
      stop_button->set_sensitive(true);
      rpm_radio->set_sensitive(true);
      trans_rot_radio->set_sensitive(true);
      trans_radio->set_sensitive(true);
      rot_radio->set_sensitive(true);
      line_trans_rot_radio->set_sensitive(true);
      orbit_radio->set_sensitive(true);
    }
  else if(behold_radio->get_active())
    {
      translation_frame->set_sensitive(false);
      rotation_frame->set_sensitive(false);
      rpm_frame->set_sensitive(false);
      orbit_frame->set_sensitive(false);
      navigator_frame->set_sensitive(false);

      send_button->set_sensitive(false);

      stop_button->set_sensitive(false);
      rpm_radio->set_sensitive(false);
      trans_rot_radio->set_sensitive(false);
      trans_radio->set_sensitive(false);
      rot_radio->set_sensitive(false);
      orbit_radio->set_sensitive(false);
      line_trans_rot_radio->set_sensitive(false);
      navigator_radio->set_sensitive(false);
    }

  if(!navigator_control && !motor_control)
    {
      reset_odometry_button->set_sensitive(false);
    }
  else
    {
      reset_odometry_button->set_sensitive(true);
    }

  if(!navigator_loaded || connection_is_dead)
    {
      behold_radio->set_active();
    }
  usleep(10000);

  if(connection_is_dead)
    {
      reset_gui();
      queue_draw();
      char str[100];
      strcpy (str,"There is no connection to a Fawkes running at ");
      strcat (str, host_name);
      strcat (str, ".\nDo you want to retry to connect to ");
      strcat (str, host_name);
      strcat (str, "?");
      Gtk::MessageDialog* dialog = new Gtk::MessageDialog((Gtk::Window&)*win, str,
                                   true, Gtk::MESSAGE_QUESTION,
                                   Gtk::BUTTONS_OK_CANCEL, true);
      dialog->set_secondary_text(
        "If not, the program will terminate.");
      dialog->set_title("Question");
      int result = dialog->run();
      delete dialog;
      switch (result)
        {
        case Gtk::RESPONSE_OK:
          printf("Try to connect.\n");
          connect();
          break;
        default:
          exit(1);
          break;
        }
    }

  queue_draw();
  return true;
}

void NavigatorGUI::on_stop_button_clicked()
{
  std::cout << "stop!" << std::endl;
  send_stop();
}


void NavigatorGUI::on_send_button_clicked()
{
  std::cout << "send" << std::endl;
  if(rpm_radio->get_active() && motor_control_radio->get_active())
    {
      navigator_rpm_message_t *rpm_msg = (navigator_rpm_message_t *)malloc(sizeof(navigator_rpm_message_t));
      rpm_msg->right = right_rpm_entry->get_value();
      rpm_msg->rear = center_rpm_entry->get_value();
      rpm_msg->left = left_rpm_entry->get_value();
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_RPM, rpm_msg, sizeof(navigator_rpm_message_t));
      net_client->enqueue(msg);
      msg->unref();
    }
  else  if(trans_rot_radio->get_active() && motor_control_radio->get_active())
    {
      navigator_trans_rot_message_t *trans_rot_msg = (navigator_trans_rot_message_t *)calloc(1, sizeof(navigator_trans_rot_message_t));
      trans_rot_msg->type_trans_rot = 1;
      trans_rot_msg->forward = xv_entry->get_value();
      trans_rot_msg->sideward = yv_entry->get_value();
      trans_rot_msg->rotation = rotation_entry->get_value();
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_TRANS_ROT, trans_rot_msg, sizeof(navigator_trans_rot_message_t));
      net_client->enqueue(msg);
      msg->unref();
    }
  else  if(trans_radio->get_active() && motor_control_radio->get_active())
    {
      navigator_trans_rot_message_t *trans_rot_msg = (navigator_trans_rot_message_t *)calloc(1, sizeof(navigator_trans_rot_message_t));
      trans_rot_msg->type_trans = 1;
      trans_rot_msg->forward = xv_entry->get_value();
      trans_rot_msg->sideward = yv_entry->get_value();
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_TRANS_ROT, trans_rot_msg, sizeof(navigator_trans_rot_message_t));
      net_client->enqueue(msg);
      msg->unref();
    }
  else  if(rot_radio->get_active() && motor_control_radio->get_active())
    {
      navigator_trans_rot_message_t *trans_rot_msg = (navigator_trans_rot_message_t *)calloc(1, sizeof(navigator_trans_rot_message_t));
      trans_rot_msg->type_rot = 1;
      trans_rot_msg->rotation = rotation_entry->get_value();
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_TRANS_ROT, trans_rot_msg, sizeof(navigator_trans_rot_message_t));
      net_client->enqueue(msg);
      msg->unref();
    }
  else  if(orbit_radio->get_active() && motor_control_radio->get_active())
    {
      navigator_orbit_message_t *orbit_msg = (navigator_orbit_message_t *)malloc(sizeof(navigator_orbit_message_t));
      orbit_msg->orbit_center_x = -mouse_point.y;
      orbit_msg->orbit_center_y = -mouse_point.x;
      orbit_msg->angular_velocity = angular_velocity_entry->get_value();
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_ORBIT, orbit_msg, sizeof(navigator_orbit_message_t));
      net_client->enqueue(msg);
      msg->unref();
    }
  else  if(line_trans_rot_radio->get_active() && motor_control_radio->get_active())
    {
      navigator_trans_rot_message_t *line_trans_rot_msg = (navigator_trans_rot_message_t *)calloc(1, sizeof(navigator_trans_rot_message_t));
      line_trans_rot_msg->type_line_trans_rot = 1;
      line_trans_rot_msg->forward = xv_entry->get_value();
      line_trans_rot_msg->sideward = yv_entry->get_value();
      line_trans_rot_msg->rotation = rotation_entry->get_value();
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_TRANS_ROT, line_trans_rot_msg, sizeof(navigator_trans_rot_message_t));
      net_client->enqueue(msg);
      msg->unref();
    }
  else if(navigator_control_radio->get_active())
    {
      send_drive_command();
    }
}

void NavigatorGUI::on_reset_odometry_button_clicked()
{
  FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_RESET_ODOMETRY);
  net_client->enqueue(msg);
  msg->unref();
}

void NavigatorGUI::on_navigator_control_radio_clicked()
{
  if(navigator_loaded)
    {
      navigator_control = true;
      if(motor_control)
        {
          navigator_unsubscribe_message_t *unsub_msg = (navigator_unsubscribe_message_t *)calloc(1, sizeof(navigator_unsubscribe_message_t));
          unsub_msg->unsub_type_motor_control = 1;
          FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_UNSUBSCRIBE, unsub_msg, sizeof(navigator_unsubscribe_message_t));
          net_client->enqueue(msg1);
          msg1->unref();
        }
      motor_control = false;

      navigator_subscribe_message_t *sub_msg = (navigator_subscribe_message_t *)calloc(1, sizeof(navigator_subscribe_message_t));
      sub_msg->sub_type_navigator_control = 1;
      FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_SUBSCRIBE, sub_msg, sizeof(navigator_subscribe_message_t));
      net_client->enqueue(msg1);
      msg1->unref();

      send_stop();

      mouse_point_mutex->lock();
      mouse_point.x = 0.;
      mouse_point.y = 0.;
      mouse_point_mutex->unlock();
    }
}


void NavigatorGUI::on_motor_control_radio_clicked()
{
  if(navigator_loaded)
    {
      motor_control = true;
      if(navigator_control)
        {
          navigator_unsubscribe_message_t *unsub_msg = (navigator_unsubscribe_message_t *)calloc(1, sizeof(navigator_unsubscribe_message_t));
          unsub_msg->unsub_type_navigator_control = 1;
          FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_UNSUBSCRIBE, unsub_msg, sizeof(navigator_unsubscribe_message_t));
          net_client->enqueue(msg1);
          msg1->unref();
        }
      navigator_control = false;

      navigator_subscribe_message_t *sub_msg = (navigator_subscribe_message_t *)calloc(1, sizeof(navigator_subscribe_message_t));
      sub_msg->sub_type_motor_control = 1;
      FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_SUBSCRIBE, sub_msg, sizeof(navigator_subscribe_message_t));
      net_client->enqueue(msg1);
      msg1->unref();
      send_stop();
    }
}


void NavigatorGUI::on_behold_radio_clicked()
{
  if(navigator_loaded)
    {
      send_stop();
      if(motor_control)
        {
          navigator_unsubscribe_message_t *unsub_msg = (navigator_unsubscribe_message_t *)calloc(1, sizeof(navigator_unsubscribe_message_t));
          unsub_msg->unsub_type_motor_control = 1;
          FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_UNSUBSCRIBE, unsub_msg, sizeof(navigator_unsubscribe_message_t));
          net_client->enqueue(msg1);
          msg1->unref();
          std::cout << "unsubscribe motor" << std::endl;
        }
      else if(navigator_control)
        {
          navigator_unsubscribe_message_t *unsub_msg = (navigator_unsubscribe_message_t *)calloc(1, sizeof(navigator_unsubscribe_message_t));
          unsub_msg->unsub_type_navigator_control = 1;
          FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_UNSUBSCRIBE, unsub_msg, sizeof(navigator_unsubscribe_message_t));
          net_client->enqueue(msg1);
          msg1->unref();
          std::cout << "unsubscribe navigator" << std::endl;
        }
    }
  motor_control = false;
  navigator_control = false;
}

bool NavigatorGUI::on_button_press_event(GdkEventButton* event)
{
  if(navigator_control)
    {
      //middle button
      if(event->button == 2)
        {
          if(obstacle_check->get_active())
            {
              Gtk::Allocation allocation = get_allocation();
              std::cout << "add obstacles" << std::endl;
              navigator_obstacle_msg_t *obstacle_msg= (navigator_obstacle_msg_t *)malloc(sizeof(navigator_obstacle_msg_t));

              obstacle_msg->x = (float) -(event->y - allocation.get_height() / 2) / zoom_factor;
              obstacle_msg->y = (float) -(event->x - allocation.get_width() / 2) / zoom_factor;
              obstacle_msg->width = 0.5f;
              std::cout << "pressed obstacle" <<  obstacle_msg->x << ", " <<  obstacle_msg->y << std::endl;

              FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_OBSTACLE, obstacle_msg, sizeof(navigator_obstacle_msg_t));
              net_client->enqueue(msg1);
              msg1->unref();
            }

        } //left button
      else if(event->button == 1)
        {
          Gtk::Allocation allocation = get_allocation();

          mouse_point_mutex->lock();
          mouse_point.x = (float) (event->x - allocation.get_width() / 2);
          mouse_point.y = (float) (event->y - allocation.get_height() / 2);
          mouse_point_mutex->unlock();
          
          if(navigator_control && orientation_check->get_active())
            {
              orientating = true;
            }
          else if(navigator_control)
            {
              orientating = false;
              orientation = 0;
              send_drive_command();
            }

        }
    }
  else if(motor_control && orbit_radio->get_active())
    {
      Gtk::Allocation allocation = get_allocation();

      mouse_point_mutex->lock();
      mouse_point.x = (float) (event->x - allocation.get_width() / 2) / zoom_factor;
      mouse_point.y = (float) (event->y - allocation.get_height() / 2) / zoom_factor;
      mouse_point_mutex->unlock();
    }

  return true;
}


bool NavigatorGUI::on_button_release_event(GdkEventButton* event)
{
  orientating = false;
  if(navigator_control && orientation_check->get_active())
    {
      send_drive_command();
      orientation = 0;
    }
  return true;
}

bool NavigatorGUI::on_motion_notify_event(GdkEventMotion* event)
{
  Gtk::Allocation allocation = get_allocation();

  cursor_point_mutex->lock();
  cursor_point.x = (float) (event->x - allocation.get_width() / 2);
  cursor_point.y = (float) (event->y - allocation.get_height() / 2);
  cursor_point_mutex->unlock();

  if(orientating)
    {
      orientation = atan2(mouse_point.y - cursor_point.y, mouse_point.x - cursor_point.x) - M_PI/2.;
    }

  return true;
}

bool NavigatorGUI::on_leave_notify_event(GdkEventCrossing* event)
{
  cursor_over_area = false;
  return true;
}


bool NavigatorGUI::on_enter_notify_event(GdkEventCrossing* event)
{
  Gtk::Allocation allocation = get_allocation();
  cursor_point_mutex->lock();
  cursor_point.x = (float) (event->x - allocation.get_width() / 2);
  cursor_point.y = (float) (event->y - allocation.get_height() / 2);
  cursor_point_mutex->unlock();
  cursor_over_area = true;
  return true;
}


void NavigatorGUI::on_zooming_value_changed()
{
  zoom_factor = zooming_adjustment->get_value() + 100;
}

void NavigatorGUI::send_stop()
{
  if(navigator_control)
    {
      navigator_velocity_message_t *velocity_msg= (navigator_velocity_message_t *)malloc(sizeof(navigator_velocity_message_t));

      velocity_msg->value = 0;

      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN,
                                  NAVIGATOR_MSGTYPE_VELOCITY, velocity_msg, sizeof(navigator_velocity_message_t));

      net_client->enqueue(msg);
      msg->unref();
    }
  else if(motor_control)
    {
      navigator_rpm_message_t *rpm_msg = (navigator_rpm_message_t *)malloc(sizeof(navigator_rpm_message_t));
      rpm_msg->right = 0.;
      rpm_msg->rear = 0.;
      rpm_msg->left = 0.;
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_RPM, rpm_msg, sizeof(navigator_rpm_message_t));
      net_client->enqueue(msg);
      msg->unref();
    }
}

void NavigatorGUI::send_drive_command()
{
  navigator_velocity_message_t *velocity_msg= (navigator_velocity_message_t *)malloc(sizeof(navigator_velocity_message_t));

  navigator_velocity_entry->update();
  velocity_msg->value = navigator_velocity_entry->get_value();
  FawkesNetworkMessage *msg2 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_VELOCITY, velocity_msg, sizeof(navigator_velocity_message_t));
  net_client->enqueue(msg2);
  msg2->unref();

  navigator_target_message_t *target_msg= (navigator_target_message_t *)malloc(sizeof(navigator_target_message_t));

  target_msg->x = -mouse_point.y / zoom_factor;
  target_msg->y = -mouse_point.x / zoom_factor; // negative because mouse_point is in screen coordinates
  target_msg->orientation = orientation;
  FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_TARGET, target_msg, sizeof(navigator_target_message_t));
  net_client->enqueue(msg1);
  msg1->unref();
}

bool NavigatorGUI::on_expose_event(GdkEventExpose* event)
{
  Glib::RefPtr<Gdk::Window> window = get_window();

  if(window)
    {
      Cairo::RefPtr<Cairo::Context> context = window->create_cairo_context();

      context->set_source_rgb(0.0, 1.0, 0.0);

      context->set_line_width(1.0);
      context->paint();

      context->set_source_rgb(0., 0., 0.);

      //draw the coordinate system
      Glib::RefPtr<Pango::Layout> layout_x = Pango::Layout::create (create_pango_context ());
      Glib::RefPtr<Pango::Layout> layout_y = Pango::Layout::create (create_pango_context ());

      layout_x->set_text("x");
      layout_y->set_text("y");
      int system_pos_x = 60;
      int system_pos_y = 60;

      Glib::RefPtr<Gdk::GC> gc = Gdk::GC::create(window);

      window->draw_layout(gc, system_pos_x - 15, system_pos_y - 50, layout_x);
      window->draw_layout(gc, system_pos_x - 43, system_pos_y - 25, layout_y);
      gc.clear();
      layout_x.clear();
      layout_y.clear();
      //x-axis
      context->move_to(system_pos_x, system_pos_y);
      context->line_to(system_pos_x, system_pos_y - 50);
      context->move_to(system_pos_x, system_pos_y - 50);
      context->line_to(system_pos_x - 5, system_pos_y - 45);
      context->move_to(system_pos_x, system_pos_y - 50);
      context->line_to(system_pos_x + 5, system_pos_y - 45);
      //y-axis
      context->move_to(system_pos_x, system_pos_y);
      context->line_to(system_pos_x - 50, system_pos_y);
      context->move_to(system_pos_x - 50, system_pos_y);
      context->line_to(system_pos_x - 45, system_pos_y - 5);
      context->move_to(system_pos_x - 50, system_pos_y);
      context->line_to(system_pos_x - 45, system_pos_y + 5);

      context->stroke();


      Gtk::Allocation allocation = get_allocation();

      const int width = allocation.get_width();
      const int height = allocation.get_height();

      //set the origin to the center of the area
      //this is also the position of the robot
      context->translate(width / 2., height / 2.);

      //draw the lines, points and obstacles
      lines.lock();
      for(LockList<NLine*>::iterator iterator = lines.begin();
          iterator != lines.end(); iterator++)
        {
          context->move_to((*iterator)->p1->x * zoom_factor, (*iterator)->p1->y * zoom_factor);
          context->line_to((*iterator)->p2->x * zoom_factor, (*iterator)->p2->y * zoom_factor);
          context->stroke();

          Obstacle *op1 = dynamic_cast<Obstacle *>((*iterator)->p1);
          if ( op1 )
            {
              // it IS an obstacle
              context->arc(op1->x * zoom_factor, op1->y * zoom_factor, op1->width / 2. * zoom_factor, 0.0, 2.0 * M_PI);
            }
          else
            {
              // it's just a point
              context->arc((*iterator)->p1->x * zoom_factor, (*iterator)->p1->y * zoom_factor, point_radius, 0.0, 2.0 * M_PI);
            }
          context->fill_preserve();
          context->stroke();
        }
      lines.unlock();

      //draw the robot
      context->arc(0, 0, robot_radius * zoom_factor, 0.0, 2.0 * M_PI);
      context->fill_preserve();
      context->stroke();

      //draw the path
      context->save();
      context->set_source_rgb(1.0, 0.0, 0.0);
      NPoint* previous_point = 0;
      std::vector<NPoint*> path;//for drawing the bezier curve
      path_points.lock();
      for(LockList<NPoint*>::iterator iterator = path_points.begin();
          iterator != path_points.end(); iterator++)
        {
          if(previous_point != 0)
            {
              context->move_to(previous_point->x * zoom_factor, previous_point->y * zoom_factor);
              context->line_to((*iterator)->x * zoom_factor, (*iterator)->y * zoom_factor);
              context->stroke();
            }
          previous_point = *iterator;
          path.push_back(*iterator);
          context->arc((*iterator)->x * zoom_factor, (*iterator)->y * zoom_factor, point_radius, 0.0, 2.0 * M_PI);
          context->fill_preserve();
          context->stroke();
        }
      context->restore();

      //draw the bezier curve
      int bezier_points_count = path.size();//the number of control points
      unsigned int i;
      //one bezier curve per bezier_points_count path_points
      for(i = 0; i + bezier_points_count - 1 < path.size(); i += bezier_points_count - 1)
        {
          double distance = 100;// fabs(path[i]->y - path[i+bezier_points_count-1]->y) * 50;

          //draw the bezier for two directions
          for(int m = 0; m < 2; m++)
            {
              if(m == 1)
                {
                  distance =  fabs(path[i]->x - path[i+bezier_points_count-1]->x) * 50;
                }
              for( int j = 0; j < distance; j++)
                {
                  double sum_x = 0;
                  double sum_y = 0;
                  for(int k = 0; k < bezier_points_count; k++)
                    {
                      // i = k
                      // n = bezier_points_count - 1
                      // t = j/distance
                      // sum^n i = 0 ( P * binoc(n, i) * pow(t, i) * pow(1. - t, n - i));
                      sum_x += path[i+k]->x * BinomialCoefficient::binoc(bezier_points_count - 1, k) * pow(j/distance, k) * pow((1. - (j/distance)), ((bezier_points_count - 1) - k));
                      sum_y += path[i+k]->y * BinomialCoefficient::binoc(bezier_points_count - 1, k) * pow(j/distance, k) * pow((1. - (j/distance)), ((bezier_points_count - 1) - k));
                    }
                  context->arc(sum_x * zoom_factor, sum_y * zoom_factor, 2, 0.0, 2.0 * M_PI);
                  context->fill_preserve();
                  context->stroke();
                }
            }

        }
      path_points.unlock();

      //draw the orbit center
      if(motor_control && orbit_radio->get_active())
        {
          context->save();
          context->set_source_rgb(1.0, 0.0, 0.0);
          mouse_point_mutex->lock();
          context->arc(mouse_point.x * zoom_factor, mouse_point.y * zoom_factor, point_radius, 0.0, 2.0 * M_PI);
          mouse_point_mutex->unlock();
          context->fill_preserve();
          context->stroke();


          context->move_to(0, 0);
          context->line_to(0, -robot_radius * 1.5);
          context->stroke();

          context->restore();
        }

      //draw the ball point
      context->save();
      context->set_source_rgb(1.0, 0.5, 0.0);
      ball_point_mutex->lock();
      context->arc(ball_point.x, ball_point.y, 0.12 * zoom_factor, 0.0, 2.0 * M_PI);
      ball_point_mutex->unlock();
      context->fill_preserve();
      context->stroke();

      //draw the odometry point
      odometry_point_mutex->lock();
      context->save();
      context->set_source_rgb(0.0, 0.0, 1.0);
      context->arc(odometry_point.x * zoom_factor, odometry_point.y * zoom_factor, point_radius, 0.0, 2.0 * M_PI);
      context->fill_preserve();
      context->stroke();

      //draw the odometry orientation
      context->set_line_width(2.5);
      odometry_orientation_mutex->lock();
      context->rotate(odometry_direction);
      odometry_orientation_mutex->unlock();
      context->set_line_cap(Cairo::LINE_CAP_ROUND);
      context->move_to(0.,0.);
      context->line_to(0., -40.);
      context->stroke();

      //draw odometry coordinates
      layout = Pango::Layout::create (create_pango_context ());
      char print_string[100];
      sprintf(print_string, "%.2f, %.2f", -odometry_point.y, -odometry_point.x);
      layout->set_text(print_string);

      Glib::RefPtr<Gdk::GC> gc_odometry = Gdk::GC::create(window);
      Gdk::Color color("blue");
      gc_odometry->set_rgb_fg_color(color);
      window->draw_layout(gc_odometry, (int) 300, (int) 20, layout);
      layout.clear();

      context->restore();
      odometry_point_mutex->unlock();

      if(debug_check->get_active())
        {
          //Debug pathfinder
          lines.lock();
          for(LockList<NLine*>::iterator iterator = lines.begin();
              iterator != lines.end(); iterator++)
            {

              gdouble middle_x;
              gdouble middle_y;

              double obstacle1_radius = 0;
              double obstacle2_radius = 0;

              Obstacle *ob1 = dynamic_cast<Obstacle *>((*iterator)->p1);
              if(ob1)
                {
                  obstacle1_radius = ob1->width / 2.;
                }
              Obstacle *ob2 = dynamic_cast<Obstacle *>((*iterator)->p2);
              if(ob2)
                {
                  obstacle2_radius = ob2->width / 2.;
                }
              double robot_width = 0.5;

              gdouble x1 = (*iterator)->p1->x;
              gdouble y1 = (*iterator)->p1->y;

              gdouble x2 = (*iterator)->p2->x;
              gdouble y2 = (*iterator)->p2->y;

              //compute the middle of the edge
              middle_x = (x1 + x2) / 2.;
              middle_y = (y1 + y2) / 2.;

              //calculate a direction vector

              //vector from p1 to p2
              double vector_x = x2 - x1;
              double vector_y = y2 - y1;

              double norm1 = sqrt(pow(vector_x, 2) + pow(vector_y, 2));

              vector_x /= norm1;
              vector_y /= norm1;

              //the point in the middle of the passage through obstacle1 and obstacle2
              NPoint * next_point1 = new NPoint(
                                       middle_x + (vector_x * (obstacle1_radius - obstacle2_radius)/2),
                                       middle_y + (vector_y * (obstacle1_radius - obstacle2_radius)/2));

              NPoint * next_point2;
              NPoint * next_point3;

              double vector_length;

              //if the edge is wide then calculate a new point further left or right of the middle
              if(distance((*iterator)->p1->x, (*iterator)->p1->y, (*iterator)->p2->x, (*iterator)->p2->y) > robot_width * 3 + obstacle1_radius + obstacle2_radius)
                {
                  vector_length = robot_width; // * 0.8;

                  //a point near by p1
                  next_point2 = new NPoint(
                                  x1 + vector_x * (vector_length + obstacle1_radius),
                                  y1 + vector_y * (vector_length + obstacle1_radius));

                  //a point near by p2
                  next_point3 = new NPoint(
                                  x2 - vector_x * (vector_length + obstacle2_radius),
                                  y2 - vector_y * (vector_length + obstacle2_radius));
                }
              else if((distance((*iterator)->p1->x, (*iterator)->p1->y, (*iterator)->p2->x, (*iterator)->p2->y) - obstacle1_radius - obstacle2_radius) <= robot_width * 3)
                {
                  next_point2 = next_point3 = next_point1;
                  //   std::cout << "middle--------------" <<distance((*iterator)->p1->x, (*iterator)->p1->y, (*iterator)->p2->x, (*iterator)->p1->y) << " <= " << robot_width * 1.5 << std::endl;
                }
              else //if the edge is narrow then calculate a new point near by the middle
                {
                  vector_length = (distance((*iterator)->p1->x, (*iterator)->p1->y, (*iterator)->p2->x, (*iterator)->p2->y) - obstacle1_radius - obstacle2_radius) / 3;

                  double vector_add = (obstacle1_radius - obstacle2_radius) / 2;

                  //a point near by the middle, placed toward to p1
                  next_point2 = new NPoint(
                                  middle_x - vector_x * (vector_length + vector_add),
                                  middle_y - vector_y * (vector_length + vector_add));

                  //a point near by the middle, placed toward to p2
                  next_point3 = new NPoint(
                                  middle_x + vector_x * (vector_length + vector_add),
                                  middle_y + vector_y * (vector_length + vector_add));
                }

              //  std::cout << "distance " << distance((*iterator)->p1->x, (*iterator)->p1->y, (*iterator)->p2->x, (*iterator)->p2->y) << std::endl;

              context->set_source_rgb(1.0, 0.0, 0.0);
              context->arc(next_point1->x * zoom_factor, next_point1->y * zoom_factor, 5, 0.0, 2.0 * M_PI);
              context->fill_preserve();
              context->stroke();
              context->arc(next_point2->x * zoom_factor, next_point2->y * zoom_factor, 5, 0.0, 2.0 * M_PI);
              context->fill_preserve();
              context->stroke();
              context->arc(next_point3->x * zoom_factor, next_point3->y * zoom_factor, 5, 0.0, 2.0 * M_PI);
              context->fill_preserve();
              context->stroke();
            }
          lines.unlock();
        }

      //draw the coordinates
      if(cursor_over_area)
        {
          context->save();
          layout = Pango::Layout::create (create_pango_context ());
          char print_string[100];
          cursor_point_mutex->lock();
          sprintf(print_string, "%.2f, %.2f", -cursor_point.y / zoom_factor, -cursor_point.x / zoom_factor);
          layout->set_text(print_string);

          Glib::RefPtr<Gdk::GC> gc = Gdk::GC::create(window);
          Gdk::Color color("white");
          gc->set_rgb_fg_color(color);
          window->draw_layout(gc, (int) (cursor_point.x + width / 2) + 5, (int) (cursor_point.y + height / 2) - 25, layout);
          layout.clear();
          cursor_point_mutex->unlock();
          context->restore();
        }

      //draw the orientation at the target
      if(navigator_control && orientating)
        {
          LockList<NPoint*>::iterator iterator = --path_points.end();
          context->set_line_width(2.5);
          context->set_line_cap(Cairo::LINE_CAP_ROUND);
          context->translate(mouse_point.x, mouse_point.y);
          context->rotate(orientation - M_PI/2.);
          context->move_to(0, 0);
          context->line_to(0.2 * zoom_factor, 0);
          context->stroke();

          if(orientating)
            {
              context->arc(0, 0, 5, 0.0, 2.0 * M_PI);
              context->fill_preserve();
              context->stroke();
            }
        }

      context.clear();

    }//if(window)


  usleep(10000);
  return true;
}

int main(int argc, char** argv)
{
  ArgumentParser *argp = new ArgumentParser(argc, argv, "r:");

  const char *host_name;

  if ( argp->has_arg("r") )
    {
      host_name = argp->arg("r");
    }
  else
    {
      host_name = "localhost";
    }

  Gtk::Main kit(argc, argv);

  NavigatorGUI gui(host_name);

  return 0;
}
