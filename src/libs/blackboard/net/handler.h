
/***************************************************************************
 *  handler.h - BlackBoard Network Handler
 *
 *  Created: Sat Mar 01 15:57:59 2008
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef __FAWKES_BLACKBOARD_NETWORK_HANDLER_H_
#define __FAWKES_BLACKBOARD_NETWORK_HANDLER_H_

#include <core/threading/thread.h>
#include <netcomm/fawkes/handler.h>

#include <core/utils/lock_queue.h>
#include <core/utils/lock_map.h>
#include <list>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Interface;
class BlackBoard;
class FawkesNetworkHub;
class BlackBoardNetHandlerInterfaceListener;
class BlackBoardNetHandlerInterfaceObserver;

class BlackBoardNetworkHandler
: public Thread,
  public FawkesNetworkHandler
{
 public:
  BlackBoardNetworkHandler(BlackBoard *blackboard,
			   FawkesNetworkHub *hub);
  ~BlackBoardNetworkHandler();

  /* from FawkesNetworkHandler interface */
  virtual void handle_network_message(FawkesNetworkMessage *msg);
  virtual void client_connected(unsigned int clid);
  virtual void client_disconnected(unsigned int clid);
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void send_opensuccess(unsigned int clid, Interface *interface);
  void send_openfailure(unsigned int clid, unsigned int error_code);


  BlackBoard *__bb;
  LockQueue< FawkesNetworkMessage * >  __inbound_queue;

  // All interfaces, key is the instance serial, value the interface
  LockMap< unsigned int, Interface * > __interfaces;
  LockMap< unsigned int, Interface * >::iterator __iit;

  std::map<unsigned int, BlackBoardNetHandlerInterfaceListener *>  __listeners;
  std::map<unsigned int, BlackBoardNetHandlerInterfaceListener *>::iterator  __lit;

  BlackBoardNetHandlerInterfaceObserver *__observer;

  // Map from instance serial to clid
  LockMap<unsigned int, unsigned int > __serial_to_clid;

  // Interfaces per client, key is the client ID, value a list of interfaces opened by client
  LockMap< unsigned int, std::list<Interface *> > __client_interfaces;
  std::list<Interface *>::iterator __ciit;

  FawkesNetworkHub *__nhub;
};

} // end namespace fawkes

#endif
