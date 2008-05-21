
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
#include <geometry/hom_vector.h>
#include <geometry/matrix.h>

namespace fawkes {
  class GameStateInterface;
  class ObjectPositionInterface;
  class WorldInfoDataContainer;
  class WorldInfoTransceiver;
}
class WorldModelNetworkThread;

class WorldModelThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ClockAspect,
  public fawkes::NetworkAspect,
  public fawkes::BlackBoardInterfaceListener,
  public fawkes::BlackBoardInterfaceObserver
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
  virtual void bb_interface_reader_added(fawkes::Interface *interface, unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(fawkes::Interface *interface, unsigned int instance_serial) throw();
  virtual void bb_interface_writer_added(fawkes::Interface *interface, unsigned int instance_serial) throw();
  virtual void bb_interface_writer_removed(fawkes::Interface *interface, unsigned int instance_serial) throw();

 private:
  class BlackboardNotificationProxy
    : public fawkes::BlackBoardInterfaceListener
  {
  public:
    BlackboardNotificationProxy(fawkes::BlackBoardInterfaceListener* listener);
    virtual ~BlackboardNotificationProxy();

    virtual void bb_interface_reader_added(fawkes::Interface *interface, unsigned int instance_serial) throw();
    virtual void bb_interface_reader_removed(fawkes::Interface *interface, unsigned int instance_serial) throw();
    virtual void bb_interface_writer_added(fawkes::Interface *interface, unsigned int instance_serial) throw();
    virtual void bb_interface_writer_removed(fawkes::Interface *interface, unsigned int instance_serial) throw();

    void add_interface(fawkes::Interface *interface);

  private:
    fawkes::BlackBoardInterfaceListener *listener;
  };

  void init_failure_cleanup();

 private:
  bool localBallPosition( fawkes::HomVector &local_ball_pos, fawkes::Matrix &local_ball_cov );
  bool globalBallPosition( bool localBallAvailable, const fawkes::HomVector &local_ball_pos, const fawkes::Matrix &local_ball_cov,
                           fawkes::HomVector &global_ball_pos, fawkes::Matrix &global_ball_cov );

  typedef std::map<unsigned int, BlackboardNotificationProxy *> ProxyMap;
  ProxyMap proxy_map;
  fawkes::LockList<BlackboardNotificationProxy *> proxy_delete_list;

  fawkes::GameStateInterface *wm_game_state_interface;
  fawkes::ObjectPositionInterface *wm_ball_interface;
  fawkes::ObjectPositionInterface *wm_pose_interface;

  fawkes::LockList<fawkes::ObjectPositionInterface *>  *in_ball_interfaces;
  fawkes::ObjectPositionInterface *in_pose_interface;
/*   LockList<fawkes::ObjectPositionInterface *>  *wm_opp_interfaces; */
/*   LockList<fawkes::ObjectPositionInterface *>  *in_opp_interfaces; */
  fawkes::LockList<fawkes::ObjectPositionInterface *>::iterator opii;

  WorldModelNetworkThread *net_thread;
  fawkes::WorldInfoTransceiver *worldinfo_sender;
  fawkes::WorldInfoDataContainer *data;
};


#endif
