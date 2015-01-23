
/***************************************************************************
 *  remote.h - Remote BlackBoard using the Fawkes network protocol
 *
 *  Created: Mon Mar 03 10:52:28 2008
 *  Copyright  2006-2015  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __BLACKBOARD_REMOTE_H_
#define __BLACKBOARD_REMOTE_H_

#include <blackboard/blackboard.h>
#include <netcomm/fawkes/client_handler.h>
#include <core/utils/lock_map.h>
#include <core/exceptions/software.h>

#include <list>

namespace fawkes {

class FawkesNetworkClient;
class FawkesNetworkMessage;
class Mutex;
class WaitCondition;
class Interface;
class InterfaceInfoList;

class BlackBoardInstanceFactory;
class BlackBoardNotifier;
class BlackBoardInterfaceProxy;
class BlackBoardInterfaceListener;
class BlackBoardInterfaceObserver;

class RemoteBlackBoard
: public BlackBoard,
  public FawkesNetworkClientHandler
{
 public:
  RemoteBlackBoard(FawkesNetworkClient *client);
  RemoteBlackBoard(const char *hostname, unsigned short int port);
  virtual ~RemoteBlackBoard();

  virtual Interface *  open_for_reading(const char *interface_type, const char *identifier,
					const char *owner = NULL);
  virtual Interface *  open_for_writing(const char *interface_type, const char *identifier,
					const char *owner = NULL);
  virtual void         close(Interface *interface);

  virtual InterfaceInfoList *  list_all();
  virtual InterfaceInfoList *  list(const char *type_pattern,
				    const char *id_pattern);
  virtual bool                 is_alive() const throw();
  virtual bool                 try_aliveness_restore() throw();

  std::list<Interface *>  open_multiple_for_reading(const char *interface_type,
						    const char *id_pattern = "*",
						    const char *owner = NULL);

  /* for FawkesNetworkClientHandler */
  virtual void          deregistered(unsigned int id) throw();
  virtual void          inbound_received(FawkesNetworkMessage *msg,
					 unsigned int id) throw();
  virtual void          connection_died(unsigned int id) throw();
  virtual void          connection_established(unsigned int id) throw();


  /* extensions for RemoteBlackBoard */

 private: /* methods */
  void        open_interface(const char *type, const char *identifier, const char *owner,
			     bool writer, Interface *iface);
  Interface * open_interface(const char *type, const char *identifier, const char *owner, bool writer);
  void        reopen_interfaces();


 private: /* members */
  Mutex *__mutex;
  FawkesNetworkClient  *__fnc;
  bool                  __fnc_owner;
  FawkesNetworkMessage *__m;
  BlackBoardInstanceFactory *__instance_factory;
  LockMap<unsigned int, BlackBoardInterfaceProxy *> __proxies;
  LockMap<unsigned int, BlackBoardInterfaceProxy *>::iterator __pit;
  std::list<BlackBoardInterfaceProxy *> __invalid_proxies;
  std::list<BlackBoardInterfaceProxy *>::iterator __ipit;

  Mutex         *__wait_mutex;
  WaitCondition *__wait_cond;

  const char *__inbound_thread;
};

} // end namespace fawkes

#endif
