
/***************************************************************************
 *  naviagtor_net_thread.h - Navigator Plugin Network Thread
 *
 *  Generated: Thu May 31 20:38:50 2007
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

#ifndef __NAVIGATOR_NAVIGATOR_NET_THREAD_H_
#define __NAVIGATOR_NAVIGATOR_NET_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/fawkes_network.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <netcomm/fawkes/handler.h>
#include <core/utils/lock_list.h>

class JoystickControl;
class MotorInterface;
class NavigatorInterface;
class NavigatorThread;
class KickerInterface;

class NavigatorNetworkThread : public Thread, public LoggingAspect, public FawkesNetworkAspect,
                               public BlackBoardAspect, public BlockedTimingAspect, public FawkesNetworkHandler, 
                               public ConfigurableAspect, public ClockAspect
{

 public:
  NavigatorNetworkThread(NavigatorThread *navigator_thread);
  virtual ~NavigatorNetworkThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  /* from FawkesNetworkHandler interface */
  virtual void handle_network_message(FawkesNetworkMessage *msg);
  virtual void client_connected(unsigned int clid);
  virtual void client_disconnected(unsigned int clid);
  virtual void process_after_loop();
  
  
 private:
  
  NavigatorThread *navigator_thread;
  MotorInterface *motor_interface;
  KickerInterface *kicker_interface;
  NavigatorInterface *navigator_interface;
  JoystickControl *joystick_control;
    
  unsigned int connected_control_client;
  LockList<unsigned int> connected_points_and_lines_clients;
  
  unsigned long int last_motor_control_thread;
    
  unsigned int logger_modulo_counter;
};


#endif
