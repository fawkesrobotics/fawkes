
/***************************************************************************
 *  client_thread.h - Thread to handle Fawkes network client
 *
 *  Created: Fri Nov 17 15:30:15 2006
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

#ifndef __NETCOMM_FAWKES_CLIENT_THREAD_H_
#define __NETCOMM_FAWKES_CLIENT_THREAD_H_

#include <core/threading/thread.h>
#include <list>

class StreamSocket;
class FawkesNetworkThread;
class FawkesNetworkMessage;
class FawkesNetworkMessageQueue;

class FawkesNetworkClientThread : public Thread
{
 public:
  FawkesNetworkClientThread(StreamSocket *s, FawkesNetworkThread *parent);
  ~FawkesNetworkClientThread();

  virtual void loop();

  unsigned int clid() const;
  void         setClientID(unsigned int client_id);

  bool alive() const;
  void enqueue(FawkesNetworkMessage *msg);

 private:

  void recv();


  unsigned int  _clid;
  unsigned int  mtu;
  void         *send_buffer;
  void         *b;
  unsigned int  buffer_size;
  bool          _alive;
  StreamSocket *s;
  FawkesNetworkThread *parent;
  FawkesNetworkMessageQueue *outbound_queue;
  FawkesNetworkMessageQueue *inbound_queue;

};

#endif
