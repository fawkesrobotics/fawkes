
/***************************************************************************
 *  joystick_tool.h - Joystick Tool
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
 
#ifndef __TOOLS_JOYSTICK_JOYSTICK_TOOL_H_
#define __TOOLS_JOYSTICK_JOYSTICK_TOOL_H_

#include <netcomm/fawkes/client_handler.h>

class FawkesNetworkClient;
class FawkesNetworkMessage;
class DatagramSocket;

class JoystickTool : public FawkesNetworkClientHandler
{
 public:
        
  JoystickTool(const char *host, bool use_udp);
  ~JoystickTool();
        
  void mainLoop();
  
 private:
        
  void deregistered();
  void inboundReceived(FawkesNetworkMessage *m);
  bool sending;

  FawkesNetworkClient *net_client;
  DatagramSocket *socket;
  bool use_udp;

  bool quit;
  float max_speed;

  float old_forward;
  float old_sideward;
  float old_rotation;
  float old_speed;
};

#endif /*JOYSTICK_TOOL_H_*/
