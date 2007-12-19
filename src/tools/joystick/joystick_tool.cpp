
/***************************************************************************
 *  joystick_tool.cpp - Joystick Tool
 *
 *  Generated: Sat Jun 02 17:45:00 2007
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id:joystick_tool.cpp 419 2007-11-08 09:45:51Z liebenberg $
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <tools/joystick/cjoystick.h>
#include <tools/joystick/joystick_tool.h>

#include <plugins/navigator/libnavi/navigator_messages.h>

#include <mainapp/plugin_messages.h>
#include <mainapp/plugin_list_message.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/message.h>
#include <netcomm/socket/socket.h>
#include <utils/system/argparser.h>
#include <utils/system/console_colors.h>

#include <iostream>
#include <unistd.h>
#include <cmath> 
#include <csignal>

/** @class JoystickTool <tools/joystick/joystick_tool.h>
 *   The joystick control for the robots.
 *   It communicates with the navigator plugin controlling the motion of the robot.
 *   
 *   Usage:
 *   >joystick_control
 *   Opens a connection to the locally running navigator plugin.
 *   >joystick_control -r 172.16.35.101
 *   Opens a connection to a running navigator plugin 
 *   on the host with the IP address 172.16.35.101.
 *   
 *   @author Martin Liebenberg
 */
/** @var JoystickTool::net_client
 *   The fawkes network client to communicate with the navigator plugin.
 */
 
/**
 * The instance of the joystick tool.
 */
JoystickTool *JoystickTool::instance = 0;

/** Signal handler.
 * For shutting down the joystick tool.
 * @param signal the signal to handle
 */
void JoystickTool::signal_handler(int signal) 
{ 
  if (signal == SIGINT || signal == SIGTERM || signal == SIGKILL) 
   {
    delete instance;
   }
}

/** Contructor.
 * @param host_name the host name of the host to connect to.
 */
JoystickTool::JoystickTool(const char *host_name)
{
  instance = this;
  net_client = new FawkesNetworkClient(host_name, 1910);
  sending = false;
  signal(SIGINT, signal_handler);
  try
    {
      net_client->connect();
    }
  catch (SocketException &e)
    {
      std::cerr << "There has to be runnig a fawkes." << std::endl;
      throw;
    }
  net_client->start();
  net_client->register_handler(this, FAWKES_CID_NAVIGATOR_PLUGIN);
  net_client->register_handler(this, FAWKES_CID_PLUGINMANAGER);
  FawkesNetworkMessage *msg1 = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_SUBSCRIBE_WATCH);
  net_client->enqueue(msg1);
  msg1->unref();
  FawkesNetworkMessage *msg2 = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
                               MSG_PLUGIN_LIST_LOADED);
  net_client->enqueue(msg2);
  msg2->unref();

  quit = false;

  // it is unlikely that all are exactly one, but it is possible that they are
  // zero. So if you start the joystick program we want to be sure that the robot
  // gets the first message, especially if it is all zero!
  old_forward = old_sideward = old_rotation = old_speed = 1.f;
}

/** Deconstructor. */
JoystickTool::~JoystickTool()
{
  quit = true;
  FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER, MSG_PLUGIN_UNSUBSCRIBE_WATCH);
  net_client->enqueue(msg);
  msg->unref();
  usleep(100000);
  printf("Got killed\n");
  net_client->disconnect();
}

/** The handler got deregistered. */
void
JoystickTool::deregistered() throw()
{
  printf("Got deregistered\n");
  quit = true;
}

void
JoystickTool::connection_established() throw()
{
  printf("Connection established\n");
}


void
JoystickTool::connection_died() throw()
{
  printf("Connection died\n");
  quit = true;
}


/** Inbound message received.
 * For receiving a subscribe error message.
 * @param m message
 */
void
JoystickTool::inbound_received(FawkesNetworkMessage *msg) throw()
{
  //    std::cerr << "receive message of type %i" << msg->msgid() << std::endl;
  if(msg->msgid() == NAVIGATOR_MSGTYPE_CONTROL_SUBERR)
    {
      std::cerr << "Error: Subscribing failed\nMaybe there is already a controlling tool subscribed." << std::endl;
      quit = true;
    }

  else if(msg->msgid() == MSG_PLUGIN_UNLOADED)
    {
      plugin_unloaded_msg_t *u = (plugin_unloaded_msg_t *)msg->payload();

      if(strncmp("navigator", u->name, PLUGIN_MSG_NAME_LENGTH) == 0)
        {
          std::cerr << "Plugin " << u->name << " unloaded." << std::endl;
          std::cerr << "Waiting for reloading navigator plugin." << std::endl;
          std::cerr << "Stops sending." << std::endl;
          sending = false;
        }
    }
  else if(msg->msgid() == MSG_PLUGIN_LOADED)
    {
      plugin_loaded_msg_t *lm= (plugin_loaded_msg_t *)msg->payload();
      if ( strncmp("navigator", lm->name, PLUGIN_MSG_NAME_LENGTH) == 0 )
        {
          std::cerr << "Navigator plugin loaded." << std::endl;
          std::cerr << "Starts sending." << std::endl;

          sending = true;

          navigator_subscribe_message_t *sub_msg = (navigator_subscribe_message_t *)calloc(1, sizeof(navigator_subscribe_message_t));
          sub_msg->sub_type_motor_control = 1;
          FawkesNetworkMessage *fmsg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_SUBSCRIBE, sub_msg, sizeof(navigator_subscribe_message_t));
          net_client->enqueue(fmsg);
          fmsg->unref();
        }
    }
  else if (msg->msgid() == MSG_PLUGIN_LOADED_LIST )
    {
      PluginListMessage *plm = msg->msgc<PluginListMessage>();
      if ( plm->has_next() )
        {
          while ( plm->has_next() )
            {
              char *p = plm->next();
              if(strcmp(p, "navigator") == 0)
                {
                  std::cerr << "Navigator plugin loaded." << std::endl;
                  std::cerr << "Starts sending." << std::endl;

                  sending = true;
                  
                  navigator_subscribe_message_t *sub_msg = (navigator_subscribe_message_t *)calloc(1, sizeof(navigator_subscribe_message_t));
                  sub_msg->sub_type_motor_control = 1;
                  FawkesNetworkMessage *fmsg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_SUBSCRIBE, sub_msg, sizeof(navigator_subscribe_message_t));
                  net_client->enqueue(fmsg);
                  fmsg->unref();
                }
              free(p);
            }
        }
      else
        {
          std::cerr << "Navigator plugin is not loaded." << std::endl;
        }
      delete plm;
    }
}


/** The main loop of the joystick control. */
void
JoystickTool::mainLoop()
{
  try
    {
      CJoystick js("/dev/input/js0");

      while( ! quit )
        {
          if(! sending)
            {
              usleep(50000);
              continue;
            }
          navigator_joystick_message_t *joy_msg = (navigator_joystick_message_t *)malloc(sizeof(navigator_joystick_message_t));
          usleep(30000);
          js.Update();
          try
            {
              //button number 5 or 8
              if(js.getButton(4) || js.getButton(7))
                {
                  float angle = atan2(js.getAxis(1),  js.getAxis(0));
                  joy_msg->forward = -js.getAxis(1) * fabs(sin(angle));    // negative = forward !!!
                  joy_msg->sideward = js.getAxis(0) * fabs(cos(angle));  // positive = rigth
                  joy_msg->rotation = js.getAxis(2);
                  joy_msg->speed = (-0.5 * js.getAxis(3) + 0.5);
                }
              else
                {
                  joy_msg->forward = 0;
                  joy_msg->sideward = 0;
                  joy_msg->rotation = 0;
                  joy_msg->speed = 0;
                }

              //button 1, the trigger
              if ( js.getButton(0) )
                {

                  navigator_kick_message_t *kick_msg = (navigator_kick_message_t *)malloc(sizeof(navigator_kick_message_t));
                  // do not kick by default
                  kick_msg->left = false;
                  kick_msg->right = false;
                  kick_msg->center = false;

                  std::cout << "axis 4: " << js.getAxis(4) << std::endl;
                  std::cout << "axis 5: " << js.getAxis(5) << std::endl;

                  if (js.getAxis(4) == -1)
                    {
                      kick_msg->left = true;
                    }
                  else if(js.getAxis(4) == 1)
                    {
                      kick_msg->right = true;
                    }
                  else if(js.getAxis(4) == 0)
                    {
                      kick_msg->right = false;
                      kick_msg->left = false;
                    }

                  if (js.getAxis(5) == -1)
                    {
                      kick_msg->left = true;
                      kick_msg->right = true;
                    }
                  else if(js.getAxis(5) == 1)
                    {
                      kick_msg->center = true;
                    }
                  else if(js.getAxis(5) == 0)
                    {
                      kick_msg->center = false;
                    }

                  if (js.getAxis(4) == 0 && js.getAxis(5) == 0)
                    {
                      std::cout << std::cred << "Use coolie hat to define kick:" << std::endl
                      << "  left:  left kick" << std::endl
                      << "  right: right kick" << std::endl
                      << "  up:    left and right kick" << std::endl
                      << "  down:  high kick" << std::cnormal << std::endl;
                    }

                  std::cerr << "Kick: left = " << kick_msg->left << "; center = " << kick_msg->center << "; right = " << kick_msg->right << std::endl;

                  FawkesNetworkMessage *kicker_msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_KICK, kick_msg, sizeof(navigator_kick_message_t));
                  net_client->enqueue(kicker_msg);
                  kicker_msg->unref();
                }

            }
          catch (Exception &e)
            {
              e.print_trace();
              throw;
            }
          //for saving network traffic
          if ( (joy_msg->forward != old_forward) ||
               (joy_msg->sideward != old_sideward) ||
               (joy_msg->rotation != old_rotation) ||
               (joy_msg->speed != old_speed) )
            {
              std::cerr << "joystick direction: " << joy_msg->forward << ", " << joy_msg->sideward << "; speed=" << joy_msg->speed;
              std::cerr << " rotation: " << joy_msg->rotation << std::endl;

              if ( ((joy_msg->forward != 0) || (joy_msg->sideward != 0)) && (joy_msg->speed == 0))
                {
                  std::cerr << std::cred << " (speed control zeroed?)" << std::cnormal << std::endl;
                }
              std::cerr << std::endl;

              FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_JOYSTICK, joy_msg, sizeof(navigator_joystick_message_t));

              net_client->enqueue(msg);
              msg->unref();

              old_forward = joy_msg->forward;
              old_sideward = joy_msg->sideward;
              old_rotation = joy_msg->rotation;
              old_speed = joy_msg->speed;
            }
        }//while
    }
  catch(Exception &e)
    {
      e.print_trace();
    }
}


/** Main function. */
int main(int argc, char **argv)
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

  JoystickTool *control = new JoystickTool(host_name);
  control->mainLoop();

  return 0;
}
