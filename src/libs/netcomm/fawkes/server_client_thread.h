
/***************************************************************************
 *  client_thread.h - Thread to handle Fawkes network client
 *
 *  Created: Fri Nov 17 15:30:15 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_FAWKES_CLIENT_THREAD_H_
#define __NETCOMM_FAWKES_CLIENT_THREAD_H_

#include <core/threading/thread.h>
#include <list>

namespace fawkes {

class StreamSocket;
class FawkesNetworkServerThread;
class FawkesNetworkMessage;
class FawkesNetworkMessageQueue;
class WaitCondition;
class Mutex;
class FawkesNetworkServerClientSendThread;

class FawkesNetworkServerClientThread : public Thread
{
 public:
  FawkesNetworkServerClientThread(StreamSocket *s, FawkesNetworkServerThread *parent);
  ~FawkesNetworkServerClientThread();

  virtual void once();
  virtual void loop();

  unsigned int clid() const;
  void         set_clid(unsigned int client_id);

  bool alive() const;
  void enqueue(FawkesNetworkMessage *msg);

  void force_send();
  void connection_died();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void recv();

  unsigned int                _clid;
  bool                        _alive;
  StreamSocket               *_s;
  FawkesNetworkServerThread  *_parent;
  FawkesNetworkMessageQueue  *_inbound_queue;

  FawkesNetworkServerClientSendThread  *_send_slave;
};

} // end namespace fawkes

#endif
