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

#include <utils/system/argparser.h>
#include <netcomm/socket/socket.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/message.h>

#include <iostream>
#include <unistd.h>

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
/** @var NavigatorGUI::control_button
 *      A Button in the GUI which activates the control of the navigator.
 */
/** @var NavigatorGUI::velocity_HScale
 *   A control to set the velocity of the robot.
 */

/** Constructor. 
 * @param host_name the host name of the host which to connect to
 */
NavigatorGUI::NavigatorGUI(const char *host_name)
{
  Glib::RefPtr<Gdk::Window> window = get_window();
  

  control_button = new Gtk::Button("Start Control");
  control = false;
   
  velocity_adjustment = new Gtk::Adjustment(0.0, 0.0, 100.0);
  velocity_HScale = new Gtk::HScale(*velocity_adjustment);
   
  velocity_adjustment->signal_value_changed().connect(sigc::mem_fun(*this,
                                                                    &NavigatorGUI::on_velocity_value_changed));
   
  //m_button.add_label("Test", false, 0.5);
  control_button->signal_clicked().connect( sigc::mem_fun(*this,
                                                          &NavigatorGUI::on_control_button_clicked) );

  
  
  target_mutex = new Mutex();
  
  value = 100;
  point_radius = 5.5;
  robot_radius = 25;
  target_point.x = 0;
  target_point.y = 0;
  old_x_coordinate = 0;
  
  robot_orientation = 1.6;
  rotate = false;
  velocity = 0;
  
  set_events(Gdk::BUTTON_PRESS_MASK);
  add_events(Gdk::BUTTON_RELEASE_MASK);
  add_events(Gdk::POINTER_MOTION_MASK);
  
  Glib::signal_idle().connect( sigc::mem_fun(*this, &NavigatorGUI::on_idle) );

  net_client = new FawkesNetworkClient(host_name, 1910);  
  try 
    {
      net_client->connect();
    } 
  catch (SocketException &e)
    {
      std::cerr << "There has to be runnig a fawkes." << std::endl;
      throw;
    }
  
  // net_client->setNoDelay(true);
  net_client->start();

  
  net_client->registerHandler(this, FAWKES_CID_NAVIGATOR_PLUGIN);
  
  navigator_subscribe_message_t *sub_msg = (navigator_subscribe_message_t *)malloc(sizeof(navigator_subscribe_message_t));
      
  sub_msg->sub_type_points_and_lines = 1;
      
  FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_SUBSCRIBE, sub_msg, sizeof(navigator_subscribe_message_t));
       
  net_client->enqueue(msg);
       
  msg->unref();
}

/** Decunstructor. */
NavigatorGUI::~NavigatorGUI()
{
  for(LockList<NPoint*>::iterator iterator = points.begin(); 
      iterator != points.end(); iterator++) 
    {
      delete *iterator;
    }
    
  points.clear();
  
  
  for(LockList<NLine*>::iterator iterator = lines.begin(); 
      iterator != lines.end(); iterator++) 
    {
      delete *iterator;
    }
  lines.clear();
    
  net_client->deregisterHandler(FAWKES_CID_NAVIGATOR_PLUGIN);
  net_client->disconnect();
  delete net_client;
  
  //delete lines
  lines.lock();
  for(LockList<NLine*>::iterator iterator = lines.begin(); 
      iterator != lines.end(); iterator++) 
    {
      delete *iterator;
    }
  lines.unlock();
    
  lines.clear();
    
  //delete points
  points.lock();
  for(LockList<NPoint*>::iterator iterator = points.begin(); 
      iterator != points.end(); iterator++) 
    {
      delete *iterator;
    }
  points.unlock();
    
  points.clear();
    
  delete target_mutex;
}

/** The handler got deregistered. */
void 
NavigatorGUI::deregistered()
{
  printf("Got deregistered\n");
}

/** Inbound mesage received.
 * @param m message
 */
void 
NavigatorGUI::inboundReceived(FawkesNetworkMessage *msg)
{
  // std::cerr << "received anything of type " << msg->msgid() << std::endl;
  if (NAVIGATOR_MSGTYPE_NODES == msg->msgid() )
    {
      NavigatorNodesListMessage *nodes_msg = msg->msgc<NavigatorNodesListMessage>();
      
      points.lock();
      
      //delete points
      for(LockList<NPoint*>::iterator iterator = points.begin(); 
          iterator != points.end(); iterator++) 
        {
          delete *iterator;
        }
     
      points.clear();
     
      //add points
      while (nodes_msg->has_next()) 
        {
          NavigatorNodesListMessage::npoint_t *p = nodes_msg->next();
           
          points.push_back(new NPoint(p->x, p->y));
        }
      points.unlock();
      
      delete nodes_msg;
    }
  
  else if (NAVIGATOR_MSGTYPE_LINES == msg->msgid() )
    {
      NavigatorLinesListMessage *lines_msg = msg->msgc<NavigatorLinesListMessage>();
      
      lines.lock();
      
      //delete lines
      for(LockList<NLine*>::iterator iterator = lines.begin(); 
          iterator != lines.end(); iterator++) 
        {
          delete *iterator;
        }
     
      lines.clear();
     
      //add lines
      while (lines_msg->has_next()) 
        {
          NavigatorLinesListMessage::nline_t *l = lines_msg->next();
           
          lines.push_back(new NLine(l->x1, l->y1, l->x2, l->y2));
        }
      lines.unlock();
      
      delete lines_msg;
     
      //   queue_draw();
    }
                
  else if(msg->msgid() == NAVIGATOR_MSGTYPE_CONTROL_SUBERR)
    {
      std::cerr << "Error: Subscribing failed.\nMaybe there is already a controlling tool subscribed." << std::endl;
      control = false;
          
      control_button->set_label("Start Control");
    }
}

bool NavigatorGUI::on_idle()
{
  //int u = 90;
  //  FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, 1, &value, sizeof(value));
  //  net_client->enqueue(msg);
    
  //net_client->wait(FAWKES_CID_NAVIGATOR_PLUGIN);->wait(FAWKES_CID_NAVIGATOR_PLUGIN);
  // net_client->wait(FAWKES_CID_NAVIGATOR_PLUGIN);
  // printf("send\n");
  // std::cout << "idle" << std::endl;
  usleep(10000);
  
  queue_draw();
  return true;
}


void NavigatorGUI::on_control_button_clicked()
{
  std::cout << "Button pressed " << std::endl;
         
  if(control) //stop control
    {
      navigator_unsubscribe_message_t *unsub_msg = (navigator_unsubscribe_message_t *)malloc(sizeof(navigator_unsubscribe_message_t));
      
      unsub_msg->unsub_type_control = 1;
      
      FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_UNSUBSCRIBE, unsub_msg, sizeof(navigator_unsubscribe_message_t));
       
      net_client->enqueue(msg1);
       
      msg1->unref();
  
  
      target_mutex->lock();
      target_point.x = 0.;
      target_point.y = 0.;
      target_mutex->unlock();
                
  
      navigator_target_message_t *target_msg= (navigator_target_message_t *)malloc(sizeof(navigator_target_message_t));
  
      target_msg->x = target_point.x;
      target_msg->y = target_point.y * -1; //because target_point is in screen coordinates
      std::cout << "pressed " <<  target_msg->x << ", " <<  target_msg->y << std::endl;
  
      FawkesNetworkMessage *msg2 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_TARGET, target_msg, sizeof(navigator_target_message_t));
      net_client->enqueue(msg2);
      msg2->unref();  
                
      navigator_velocity_message_t *velocity_msg= (navigator_velocity_message_t *)malloc(sizeof(navigator_velocity_message_t));
  
      velocity_msg->value = 0.;
   
      FawkesNetworkMessage *msg3 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_VELOCITY, velocity_msg, sizeof(navigator_velocity_message_t));
      net_client->enqueue(msg3);
      msg3->unref();  
  
      control_button->set_label("Start Control");
      control = false;
    }
  else //start control
    {
  
      navigator_subscribe_message_t *sub_msg = (navigator_subscribe_message_t *)malloc(sizeof(navigator_subscribe_message_t));
      
      sub_msg->sub_type_control = 1;
      
      FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_SUBSCRIBE, sub_msg, sizeof(navigator_subscribe_message_t));
       
      net_client->enqueue(msg1);
       
      msg1->unref();
                
      navigator_velocity_message_t *velocity_msg= (navigator_velocity_message_t *)malloc(sizeof(navigator_velocity_message_t));
  
      velocity_msg->value = velocity_adjustment->get_value();
   
      FawkesNetworkMessage *msg2 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_VELOCITY, velocity_msg, sizeof(navigator_velocity_message_t));
      net_client->enqueue(msg2);
      msg2->unref();  
                
      control_button->set_label("Stop Control");
      control = true;
    }
}

bool NavigatorGUI::on_button_press_event(GdkEventButton* event)
{
  // std::cout << "pressed " << event->x << ", " << event->y << std::endl;
  // std::cout << "pressed button" << event->button << std::endl;
  if(control)
    {
        
      //right button
      if(event->button == 3)
        {
          //  std::cout << "pressed right" << std::endl;
          rotate = true;
          old_x_coordinate = (int)event->x;
        }
      else
        {
          Gtk::Allocation allocation = get_allocation();
  
          target_mutex->lock();
          target_point.x = (float) (event->x - allocation.get_width() / 2.);
          target_point.y = (float) (event->y - allocation.get_height() / 2.);
          target_mutex->unlock();
  
          navigator_target_message_t *target_msg= (navigator_target_message_t *)malloc(sizeof(navigator_target_message_t));
  
          target_msg->x = target_point.x;
          target_msg->y = target_point.y * -1; //because target_point is in screen coordinates
          std::cout << "pressed " <<  target_msg->x << ", " <<  target_msg->y << std::endl;
  
          FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_TARGET, target_msg, sizeof(navigator_target_message_t));
          net_client->enqueue(msg1);
          msg1->unref();  
  
          navigator_velocity_message_t *velocity_msg= (navigator_velocity_message_t *)malloc(sizeof(navigator_velocity_message_t));
  
          velocity_msg->value = velocity;
  
          FawkesNetworkMessage *msg2 = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_TARGET, velocity_msg, sizeof(navigator_velocity_message_t));
          net_client->enqueue(msg2);
          msg2->unref();  
        }
    }
  // queue_draw();
  //Gtk::Widget::on_button_press_event(event);
  
  return true;
}


bool NavigatorGUI::on_button_release_event(GdkEventButton* event)
{
  std::cout << "button release" << std::endl;
  
  if(control && event->button == 3)
    {
      rotate = false;
    }
  
  return true;
}
 
bool NavigatorGUI::on_motion_notify_event(GdkEventMotion* event)
{
  
  if(rotate)
    {
      robot_orientation += (event->x - old_x_coordinate) / 150;
      old_x_coordinate = (int)event->x;
    }
  
  return true;
}


void NavigatorGUI::on_velocity_value_changed()
{
  std::cout << "value changed" << velocity_adjustment->get_value() << std::endl;
  
  if(control)
    {
  
  
      navigator_velocity_message_t *velocity_msg= (navigator_velocity_message_t *)malloc(sizeof(navigator_velocity_message_t));
  
      velocity_msg->value = velocity_adjustment->get_value();
      velocity = velocity_adjustment->get_value();
   
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_VELOCITY, velocity_msg, sizeof(navigator_velocity_message_t));
      net_client->enqueue(msg);
      msg->unref();  
    }
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
        
      Gtk::Allocation allocation = get_allocation();
        
      const int width = allocation.get_width();
      const int height = allocation.get_height();
         
      //set the origin to the center of the area
      //this is also the position of the robot
      context->translate(width / 2., height / 2.);
    
      //draw the points
      points.lock();
      for(LockList<NPoint*>::iterator iterator = points.begin(); 
          iterator != points.end(); iterator++) 
        {
          context->arc((*iterator)->x, (*iterator)->y, point_radius, 0.0, 2.0 * M_PI);   
          context->fill_preserve();
          context->stroke();
        }
      points.unlock();
    
      //draw the lines
      lines.lock();
      for(LockList<NLine*>::iterator iterator = lines.begin(); 
          iterator != lines.end(); iterator++) 
        {
          context->move_to((*iterator)->p1.x, (*iterator)->p1.y);
          context->line_to((*iterator)->p2.x, (*iterator)->p2.y);
          context->stroke();
        }
      lines.unlock();
    
      //draw the robot
      context->arc(0, 0, robot_radius, 0.0, 2.0 * M_PI);   
      context->fill_preserve();
      context->stroke();
    
      if(control)
        {
          //draw the target point
          context->save();
          context->set_source_rgb(1.0, 0.0, 0.0);
          target_mutex->lock();
          context->arc(target_point.x, target_point.y, point_radius, 0.0, 2.0 * M_PI);   
          target_mutex->unlock();
          context->fill_preserve();
          context->stroke();
    
          context->rotate(robot_orientation);
    
          context->move_to(0, 0);
          context->line_to(0, -robot_radius * 1.5);
          context->stroke();
    
    
    
          context->restore();
    
    
        }
    
    }
  
  
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

  Gtk::Window win;
  win.set_title("Navigator GUI");
  win.resize(800, 800);         
   

  NavigatorGUI tester(host_name);
   
   
  // Gtk::Button m_button;
   
  // m_button.add_label("Test", false, 0.5);
  //win.add(m_button);
  //win.add(tester);
  /*
    Gtk::Adjustment m_adjustment(0.0, 0.0, 100.0);
    m_adjustment.signal_value_changed().connect(sigc::mem_fun(*this,
    &Navigator::on_adjustment_value_changed));
    Gtk::HScale m_HScale(m_adjustment);
  */
  Gtk::VBox m_VBox_Main;
  Gtk::ButtonBox* bbox = 0;

  bbox = Gtk::manage( new Gtk::HButtonBox() );

  bbox->set_border_width(5);


  /* Set the appearance of the Button Box */
  bbox->set_layout(Gtk::BUTTONBOX_SPREAD);
  bbox->set_spacing(30);

  bbox->add(*tester.velocity_HScale);
  bbox->add(*tester.control_button);

  m_VBox_Main.pack_start(*bbox, Gtk::PACK_SHRINK, 4);
  m_VBox_Main.pack_start(tester, Gtk::PACK_EXPAND_WIDGET);
  win.add(m_VBox_Main);
  // tester.show();
   
   
  win.show_all_children();
   
  Gtk::Main::run(win);

  return 0;
}
