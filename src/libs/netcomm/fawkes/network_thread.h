
/***************************************************************************
 *  network_thread.h - Thread to manage Fawkes network clients
 *
 *  Created: Sun Nov 19 14:27:31 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __NETCOMM_FAWKES_CLIENT_MANAGER_THREAD_H_
#define __NETCOMM_FAWKES_CLIENT_MANAGER_THREAD_H_

#include <core/threading/thread.h>
#include <netcomm/fawkes/emitter.h>

#include <map>

class ThreadCollector;
class Mutex;
class WaitCondition;
class FawkesNetworkClientThread;
class FawkesNetworkAcceptorThread;
class FawkesNetworkHandler;
class FawkesNetworkMessage;
class FawkesNetworkMessageQueue;

class FawkesNetworkThread : public Thread, public FawkesNetworkEmitter
{
 public:
  FawkesNetworkThread(ThreadCollector *thread_collector,
		      unsigned int fawkes_port);
  virtual ~FawkesNetworkThread();

  virtual void loop();

  void add_handler(FawkesNetworkHandler *handler);
  void remove_handler(FawkesNetworkHandler *handler);

  void add_client(FawkesNetworkClientThread *client);

  void wakeup();

  void dispatch(FawkesNetworkMessage *msg);
  void broadcast(FawkesNetworkMessage *msg);
  void send(FawkesNetworkMessage *msg);

  void process();

 private:
  ThreadCollector     *thread_collector;

  Mutex               *handlers_mutex;
  Mutex               *clients_mutex;
  Mutex               *wait_mutex;
  WaitCondition       *wait_cond;

  unsigned int         next_client_id;

  FawkesNetworkAcceptorThread *acceptor_thread;

  // key: component id,  value: handler
  std::map<unsigned int, FawkesNetworkHandler *> handlers;
  std::map<unsigned int, FawkesNetworkHandler *>::iterator hit;

  // key: client id,     value: client thread
  std::map<unsigned int, FawkesNetworkClientThread *> clients;
  std::map<unsigned int, FawkesNetworkClientThread *>::iterator cit;

  FawkesNetworkMessageQueue *inbound_messages;
};

#endif
