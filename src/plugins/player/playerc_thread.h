
/***************************************************************************
 *  playerc_thread.h - Thread that handles the Player client connection
 *
 *  Created: Mon Aug 11 22:40:45 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PLAYER_PLAYERC_THREAD_H_
#define __PLUGINS_PLAYER_PLAYERC_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>
#include <aspect/network.h>

#include <string>
#include <map>
#include <list>

namespace PlayerCc {
  class PlayerClient;
  class ClientProxy;
}

namespace fawkes {
  class ObjectPositionInterface;
}

class PlayerProxyFawkesInterfaceMapper;

class PlayerClientThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::NetworkAspect
{
 public:
  /** Map for Fawkes interfaces. */
  typedef std::map<std::string, fawkes::Interface *>      InterfaceMap;

  /** Map for Player interfaces. */
  typedef std::map<std::string, PlayerCc::ClientProxy *>  ProxyMap;

  /** Map for proxy-interface mappers. */
  typedef std::list<PlayerProxyFawkesInterfaceMapper *>   MapperList;

  PlayerClientThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  void sync_fawkes_to_player();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void open_fawkes_interfaces();
  void open_player_proxies();
  void create_mappers();

  void close_fawkes_interfaces();
  void close_player_proxies();

 private:
  PlayerCc::PlayerClient    *__client;

  std::string  __cfg_player_host;
  unsigned int __cfg_player_port;

  InterfaceMap __imap;
  ProxyMap     __pmap;
  MapperList   __mappers;
};


#endif
