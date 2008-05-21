
/***************************************************************************
 *  navigator_net_thread.h - Navigator Plugin Network Thread
 *
 *  Generated: Thu May 31 20:38:50 2007
 *  Copyright  2007  Martin Liebenberg
 *             2007  Tim Niemueller
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
#include <core/utils/lock_queue.h>

#include <netinet/in.h>

class JoystickControl;
class NavigatorThread;

namespace fawkes {
  class MotorInterface;
  class NavigatorInterface;
  class KickerInterface;
  class Time;
}

class NavigatorNetworkThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::FawkesNetworkAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::FawkesNetworkHandler, 
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect
{

 public:
  NavigatorNetworkThread(NavigatorThread *navigator_thread);
  virtual ~NavigatorNetworkThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  /* from FawkesNetworkHandler interface */
  virtual void handle_network_message(fawkes::FawkesNetworkMessage *msg);
  virtual void client_connected(unsigned int clid);
  virtual void client_disconnected(unsigned int clid);

 private:
  void process_network_message(fawkes::FawkesNetworkMessage *msg);


  NavigatorThread *navigator_thread;
  JoystickControl *joystick_control;

  fawkes::MotorInterface     *motor_interface;
  fawkes::KickerInterface    *kicker_interface;
  fawkes::NavigatorInterface *navigator_interface;
    
  unsigned int connected_control_client;
  fawkes::LockList<unsigned int> connected_points_and_lines_clients;
  fawkes::LockList<unsigned int> connected_odometry_clients;
  fawkes::LockList<unsigned int> connected_ball_clients;
  
  unsigned long int last_motor_controller;
  char *            last_motor_controller_thread_name;
    
  unsigned int logger_modulo_counter;

  fawkes::LockQueue< fawkes::FawkesNetworkMessage * > inbound_queue;

  float sending_pause;
  fawkes::Time sending_time;
};


#endif
