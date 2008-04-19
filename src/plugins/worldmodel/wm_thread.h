
/***************************************************************************
 *  wm_thread.h - Fawkes WorldModel Plugin Thread
 *
 *  Created: Fri Jun 29 11:54:58 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WORLDMODEL_WM_THREAD_H_
#define __PLUGINS_WORLDMODEL_WM_THREAD_H_

#include <core/threading/thread.h>
#include <core/utils/lock_list.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/network.h>
#include <aspect/clock.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>


class GameStateInterface;
class ObjectPositionInterface;
class WorldInfoDataContainer;
class WorldModelNetworkThread;
class WorldInfoTransceiver;

class WorldModelThread
: public Thread,
  public BlockedTimingAspect,
  public LoggingAspect,
  public ConfigurableAspect,
  public BlackBoardAspect,
  public ClockAspect,
  public NetworkAspect,
  public BlackBoardInterfaceListener,
  public BlackBoardInterfaceObserver
{
 public:
  WorldModelThread(WorldModelNetworkThread *net_thread);
  virtual ~WorldModelThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  // interface observer
  virtual void bb_interface_created(const char *type, const char *id) throw();
  virtual void bb_interface_destroyed(const char *type, const char *id) throw();

  // interface listener
  virtual void bb_interface_reader_added(Interface *interface, unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(Interface *interface, unsigned int instance_serial) throw();
  virtual void bb_interface_writer_added(Interface *interface, unsigned int instance_serial) throw();
  virtual void bb_interface_writer_removed(Interface *interface, unsigned int instance_serial) throw();

 private:
  class BlackboardNotificationProxy : public BlackBoardInterfaceListener
  {
  public:
    BlackboardNotificationProxy(BlackBoardInterfaceListener* listener);
    virtual ~BlackboardNotificationProxy();
    
    virtual void bb_interface_reader_added(Interface *interface, unsigned int instance_serial) throw();
    virtual void bb_interface_reader_removed(Interface *interface, unsigned int instance_serial) throw();
    virtual void bb_interface_writer_added(Interface *interface, unsigned int instance_serial) throw();
    virtual void bb_interface_writer_removed(Interface *interface, unsigned int instance_serial) throw();
    
    void add_interface(Interface *interface);
    
  private:
    BlackBoardInterfaceListener *listener;
  };

  void init_failure_cleanup();

 private:
  typedef std::map<unsigned int, BlackboardNotificationProxy *> ProxyMap;
  ProxyMap proxy_map;
  LockList<BlackboardNotificationProxy *> proxy_delete_list;

  GameStateInterface *wm_game_state_interface;
  ObjectPositionInterface *wm_ball_interface;
  ObjectPositionInterface *wm_pose_interface;

  LockList<ObjectPositionInterface *>  *in_ball_interfaces;
  ObjectPositionInterface *in_pose_interface;
/*   LockList<ObjectPositionInterface *>  *wm_opp_interfaces; */
/*   LockList<ObjectPositionInterface *>  *in_opp_interfaces; */
  LockList<ObjectPositionInterface *>::iterator opii;

  WorldModelNetworkThread *net_thread;
  WorldInfoTransceiver *worldinfo_sender;
  WorldInfoDataContainer *data;
};


#endif
