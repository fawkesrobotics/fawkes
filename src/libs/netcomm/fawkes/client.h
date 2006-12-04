
/***************************************************************************
 *  client.h - Fawkes network client
 *
 *  Created: Tue Nov 21 18:42:10 2006
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

#ifndef __NETCOMM_FAWKES_CLIENT_H_
#define __NETCOMM_FAWKES_CLIENT_H_

#include <netcomm/fawkes/message_queue.h>
#include <netcomm/fawkes/message.h>
#include <netcomm/fawkes/component_ids.h>

class StreamSocket;

class FawkesNetworkClient
{
 public:
  FawkesNetworkClient(const char *host, unsigned short int port);
  ~FawkesNetworkClient();

  void connect();
  void disconnect();

  void enqueue(FawkesNetworkMessage *message);
  void send();
  void recv();
  void set_nodelay(bool nodelay);
  bool nodelay();

  void wait();

  FawkesNetworkMessageQueue *  inbound_queue();

 private:
  const char *hostname;
  unsigned short int port;

  void *buffer;
  unsigned int buffer_size;

  StreamSocket *s;

  FawkesNetworkMessageQueue *  inbound_msgq;
  FawkesNetworkMessageQueue *  outbound_msgq;
};


#endif
