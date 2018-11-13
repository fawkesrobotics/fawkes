
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

#ifndef _FAWKES_BLACKBOARD_NETWORK_HANDLER_H_
#define _FAWKES_BLACKBOARD_NETWORK_HANDLER_H_

#include <core/threading/thread.h>
#include <netcomm/fawkes/handler.h>

#include <core/utils/lock_queue.h>
#include <core/utils/lock_map.h>
#include <list>

namespace fawkes {

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


  BlackBoard *bb_;
  LockQueue< FawkesNetworkMessage * >  inbound_queue_;

  // All interfaces, key is the instance serial, value the interface
  LockMap< unsigned int, Interface * > interfaces_;
  LockMap< unsigned int, Interface * >::iterator iit_;

  std::map<unsigned int, BlackBoardNetHandlerInterfaceListener *>  listeners_;
  std::map<unsigned int, BlackBoardNetHandlerInterfaceListener *>::iterator  lit_;

  BlackBoardNetHandlerInterfaceObserver *observer_;

  // Map from instance serial to clid
  LockMap<unsigned int, unsigned int > serial_to_clid_;

  // Interfaces per client, key is the client ID, value a list of interfaces opened by client
  LockMap< unsigned int, std::list<Interface *> > client_interfaces_;
  std::list<Interface *>::iterator ciit_;

  FawkesNetworkHub *nhub_;
};

} // end namespace fawkes

#endif
