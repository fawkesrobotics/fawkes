
/***************************************************************************
 *  client.cpp - Fawkes network client
 *
 *  Created: Tue Nov 21 18:44:58 2006
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

#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/message_queue.h>
#include <netcomm/fawkes/transceiver.h>
#include <netcomm/socket/stream.h>

/** @class FawkesNetworkClient netcomm/fawkes/client.h
 * Simple Fawkes network client. Allows access to a remote instance via the
 * network. Encapsulates all needed interaction with the network.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param hostname remote host to connect to.
 * @param port port to connect to.
 */
FawkesNetworkClient::FawkesNetworkClient(const char *hostname, unsigned short int port)
{
  inbound_msgq = new FawkesNetworkMessageQueue();
  outbound_msgq = new FawkesNetworkMessageQueue();
  s = new StreamSocket();
  this->hostname = hostname;
  this->port     = port;

  buffer = NULL;
  buffer_size = 0;
}


/** Destructor. */
FawkesNetworkClient::~FawkesNetworkClient()
{
  delete inbound_msgq;
  delete outbound_msgq;
  if ( buffer != NULL ) {
    free(buffer);
    buffer = NULL;
    buffer_size = 0;
  }
  delete s;
}


/** Connect to remote.
 * @exception SocketException thrown by Socket::connect()
 */
void
FawkesNetworkClient::connect()
{
  try {
    s->connect(hostname, port);
    buffer_size = s->mtu() - 40;
    buffer = malloc(buffer_size);
  } catch (SocketException &e) {
    throw;
  }
}


/** Disconnect socket. */
void
FawkesNetworkClient::disconnect()
{
  delete s;
  s = new StreamSocket();
}


/** Enqueue message to send.
 * @param message message to send
 */
void
FawkesNetworkClient::enqueue(FawkesNetworkMessage *message)
{
  message->ref();
  outbound_msgq->push_locked(message);
}


/** Send queued messages. */
void
FawkesNetworkClient::send()
{
  FawkesNetworkTransceiver::send(s, outbound_msgq, buffer, buffer_size);
}


/** Receive messages. */
void
FawkesNetworkClient::recv()
{
  while ( s->available() ) {
    FawkesNetworkTransceiver::recv(s, inbound_msgq);
  }
}


/** Check nodelay of socket.
 * @return true, if nodelay is enabled, false otherwise.
 * @see StreamSocket::nodelay()
 */
bool
FawkesNetworkClient::nodelay()
{
  return s->nodelay();
}


/** Set nodelay option.
 * @param nodelay true to enable nodelay, false otherwise.
 * @see StreamSocket::set_nodelay()
 * @see StreamSocket::nodelay()
 */
void
FawkesNetworkClient::set_nodelay(bool nodelay)
{
  s->set_nodelay(nodelay);
}


/** Get inbound message queue.
 * After a call to recv() all message that could be read at that time
 * are in this queue. Note that you have to look the queue while working
 * on it!
 * @return inbound message queue.
 */
FawkesNetworkMessageQueue *
FawkesNetworkClient::inbound_queue()
{
  return inbound_msgq;
}


/** Wait until inbound messages have been receive or the connection dies.
 */
void
FawkesNetworkClient::wait()
{
  short p = 0;
  do {
    try {
      p = s->poll();
    } catch (Exception &e) {
      // make sure we abort waiting
      p = Socket::POLL_IN;
    }
    /*
    if ( p & Socket::POLL_IN )     printf("POLL_IN\n");
    if ( p & Socket::POLL_OUT )    printf("POLL_OUT\n");
    if ( p & Socket::POLL_PRI )    printf("POLL_PRI\n");
    if ( p & Socket::POLL_HUP )    printf("POLL_HUP\n");
    if ( p & Socket::POLL_RDHUP )  printf("POLL_RDHUP\n");
    if ( p & Socket::POLL_ERR )    printf("POLL_ERR\n");
    */
  } while ( ! (p & Socket::POLL_IN) );
}
