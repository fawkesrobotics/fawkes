
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
 
#ifndef __TOOLS_JOYSTICK_CONTROL_JOYSTICK_TOOL_H_
#define __TOOLS_JOYSTICK_CONTROL_JOYSTICK_TOOL_H_

#include <netcomm/fawkes/client_handler.h>

class FawkesNetworkClient;
class FawkesNetworkMessage;
class NetworkConfiguration;

class JoystickTool : public FawkesNetworkClientHandler
{
 public:
        
  JoystickTool(const char *host);
        
  ~JoystickTool();
        
  void mainLoop();
    
    
 private:
        
  FawkesNetworkClient *net_client;
        
  NetworkConfiguration *net_config;
        
  void deregistered();
        
  void inboundReceived(FawkesNetworkMessage *m);
        
  bool sending;
        
};

#endif /*JOYSTICK_TOOL_H_*/
