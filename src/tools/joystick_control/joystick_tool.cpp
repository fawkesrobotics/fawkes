
/***************************************************************************
 *  joystick_tool.cpp - Joystick Tool
 *
 *  Generated: Sat Jun 02 17:45:00 2007
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
 
#include <tools/joystick_control/joystick.h>
#include <tools/joystick_control/joystick_tool.h>

#include <plugins/navigator/libnavi/navigator_messages.h>

#include <mainapp/plugin_messages.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/message.h>
#include <netcomm/socket/socket.h>
#include <utils/system/argparser.h>
#include <config/netconf.h>

#include <iostream>
#include <unistd.h>
#include <cmath>

//#define NO_JOYSTICK

/** @class JoystickTool <tools/joystick_control/joystick_tool.h>
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

/** Contructor.
 * @param host_name the host name of the host to connect to.
 */
JoystickTool::JoystickTool(const char *host_name)
{
  net_client = new FawkesNetworkClient(host_name, 1910);
  
  sending = false;
  
  try 
    {
      net_client->connect();
    } 
  catch (SocketException &e)
    {
      std::cerr << "There has to be runnig a fawkes." << std::endl;
      throw;
    }
  
  net_config = new NetworkConfiguration(net_client);
  
  net_config->setMirrorMode(true);
  
  net_client->start();
  
  net_client->registerHandler(this, FAWKES_CID_NAVIGATOR_PLUGIN);

  net_client->registerHandler(this, FAWKES_CID_PLUGINMANAGER);
}        

/** Deconstructor. */
JoystickTool::~JoystickTool()
{
  net_client->disconnect();
  
  delete net_client;
  
  delete net_config;
}

/** The handler got deregistered. */
void 
JoystickTool::deregistered()
{
  printf("Got deregistered\n");
  exit(0);
}

/** Inbound message received.
 * For receiving a subscribe error message.
 * @param m message
 */
void 
JoystickTool::inboundReceived(FawkesNetworkMessage *msg)
{
  //    std::cerr << "receive message of type %i" << msg->msgid() << std::endl;
  if(msg->msgid() == NAVIGATOR_MSGTYPE_CONTROL_SUBERR)
    {
      std::cerr << "Error: Subscribing failed\nMaybe there is already a controlling tool subscribed." << std::endl;
      
      exit(1);
    }
     
  else if(msg->msgid() == MSG_PLUGIN_UNLOADED)
    {
      plugin_unloaded_msg_t *u = (plugin_unloaded_msg_t *)msg->payload();
      
      if(strcmp("navigator", u->name) == 0)
        {
          std::cerr << "Plugin " << u->name << " unloaded." << std::endl;
          std::cerr << "Waiting for reloading navigator plugin." << std::endl;
          std::cerr << "Stops sending." << std::endl;
          sending = false;
        }
    }
  else if(msg->msgid() == MSG_PLUGIN_LOADED)
    {  
      std::cerr << "Navigator plugin loaded." << std::endl;
      std::cerr << "Starts sending." << std::endl;
      
      sending = true;
      
      navigator_subscribe_message_t *sub_msg = (navigator_subscribe_message_t *)malloc(sizeof(navigator_subscribe_message_t));
      
      sub_msg->sub_type_control = 1;
      
      FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_SUBSCRIBE, sub_msg, sizeof(navigator_subscribe_message_t));
       
      net_client->enqueue(msg);
       
      msg->unref();
    }
}


/** The main loop of the joystick control. */
void 
JoystickTool::mainLoop()
{
  try
    {
#ifndef NO_JOYSTICK
      CJoystick js("/dev/input/js0");
#endif
      while(1)
        {
          navigator_joystick_message_t *joy_msg = (navigator_joystick_message_t *)malloc(sizeof(navigator_joystick_message_t));
          navigator_kick_message_t *kick_msg = (navigator_kick_message_t *)malloc(sizeof(navigator_kick_message_t));
      
#ifndef NO_JOYSTICK
          usleep(50000); 
          js.Update();
      
          std::cout << "axis 4: " << js.getAxis(4) << std::endl;
          std::cout << "axis 5: " << js.getAxis(5) << std::endl;
      
      
          try
            {
        
              float max_speed = net_config->get_float("joystick_tool", "/max_speed");
       
              std::cout << "max speed" << max_speed << std::endl;
      
              if(js.getButton(4) || js.getButton(7)) //button number 5 or 8
                {
                  float angle = atan2(js.getAxis(1),  js.getAxis(0));
     
                  joy_msg->forward = js.getAxis(1) * fabs(sin(angle));    // negative = forward !!!
                  joy_msg->sideward = js.getAxis(0) * fabs(cos(angle));  // positive = rigth
                  joy_msg->rotation = js.getAxis(2);
                  joy_msg->speed = max_speed * (-0.5 * js.getAxis(3) + 0.5);
       
                }
              else
                {
                  joy_msg->forward = 0;
                  joy_msg->sideward = 0;
                  joy_msg->rotation = 0;
                  joy_msg->speed = 0;
        
                  kick_msg->left = false;
                  kick_msg->right = false;
                  kick_msg->center = false;
                }  
       
              if(js.getButton(0)) //button 1, the trigger
                {
                  if(js.getAxis(4) == -1)
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
         
                  if(js.getAxis(5) == -1)
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
         
                  if(js.getAxis(4) == 0 && js.getAxis(5) == 0)
                    {          
                      kick_msg->left = true;
                      kick_msg->right = true;
                      kick_msg->center = false;
                    }
                }
       
            }  
          catch (Exception &e) 
            {
              e.printTrace();
              throw;
            }
       
          std::cerr << "joystick: " << js.getAxis(1) << ", " << js.getAxis(0) << ", "<< js.getAxis(2) << ", " << js.getAxis(3) <<  "," << joy_msg->speed << std::endl;
      
          std::cerr << "joystick direction: " << joy_msg->forward << ", " << joy_msg->sideward  << std::endl;
      
          std::cerr << "Kick: left = " << kick_msg->left << "; center = " << kick_msg->center << "; right = " << kick_msg->right << std::endl;
#endif
#ifdef NO_JOYSTICK

          joy_msg->forward = 0.6;    // negative = forward !!!
          joy_msg->sideward = 0.5;  // positive = rigth
          joy_msg->rotation = 0.;
          joy_msg->speed = 0.;
          usleep(1000);
#endif
      
          if(sending)
            {
              FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_JOYSTICK, joy_msg, sizeof(navigator_joystick_message_t));
       
              net_client->enqueue(msg);
       
              msg->unref();
       
       
              FawkesNetworkMessage *kicker_msg = new FawkesNetworkMessage(FAWKES_CID_NAVIGATOR_PLUGIN, NAVIGATOR_MSGTYPE_KICK, kick_msg, sizeof(navigator_kick_message_t));
       
              net_client->enqueue(kicker_msg);
       
              kicker_msg->unref();
            }
        }//while
    }
  catch(char const*ex)
    {
      std::cerr << ex << std::endl;
    }
}//mainLoop


/** Main function. */
int main(int argc, char **argv) 
{
  ArgumentParser *argp = new ArgumentParser(argc, argv, "r:");
  
  char *host_name;
  
  if ( argp->hasArgument("r") ) 
    {
      host_name = argp->getArgument("r");
    } 
  else 
    {
      host_name = "localhost";
    }
  
  JoystickTool control(host_name);
  
  control.mainLoop(); 
  
  return 0;     
}

