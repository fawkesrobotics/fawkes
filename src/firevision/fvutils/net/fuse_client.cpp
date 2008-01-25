
/***************************************************************************
 *  fuse_client.cpp - FUSE network transport client
 *
 *  Created: Thu Mar 29 00:47:24 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <fvutils/net/fuse_client.h>

#include <fvutils/net/fuse_transceiver.h>
#include <fvutils/net/fuse_message_queue.h>
#include <fvutils/net/fuse_message.h>
#include <fvutils/net/fuse_client_handler.h>

#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>
#include <core/exceptions/software.h>
#include <netcomm/socket/stream.h>
#include <netcomm/utils/exceptions.h>

#include <cstring>
#include <netinet/in.h>

/** @class FuseClient <fvutils/net/fuse_client.h>
 * FUSE client.
 * FUSE is the FireVision protocol to retrieve information, images and lookup
 * tables from vision processes and to send control commands to these systems.
 * The client is used in the retrieving or controlling process.
 * @ingroup FUSE
 * @ingroup FireVision
 * @author Tim Niemueller
 */

/** Constructor.
 * @param hostname host to connect to
 * @param port port to connect to
 * @param handler client handler to handle incoming data
 */
FuseClient::FuseClient(const char *hostname, unsigned short int port,
		       FuseClientHandler *handler)
  : Thread("FuseClient")
{
  __hostname = strdup(hostname);
  __port = port;
  __handler = handler;

  __wait_timeout = 10;

  __inbound_msgq = new FuseNetworkMessageQueue();
  __outbound_msgq = new FuseNetworkMessageQueue();

  __mutex = new Mutex();
  __waitcond = new WaitCondition();
  __socket = new StreamSocket();

  __alive = true;
}


/** Destructor. */
FuseClient::~FuseClient()
{
  free(__hostname);

  while ( ! __inbound_msgq->empty() ) {
    FuseNetworkMessage *m = __inbound_msgq->front();
    m->unref();
    __inbound_msgq->pop();
  }
  delete __inbound_msgq;

  while ( ! __outbound_msgq->empty() ) {
    FuseNetworkMessage *m = __outbound_msgq->front();
    m->unref();
    __outbound_msgq->pop();
  }
  delete __outbound_msgq;

  delete __mutex;
  delete __waitcond;
  delete __socket;
}


/** Connect. */
void
FuseClient::connect()
{
  __socket->connect(__hostname, __port);

  FUSE_greeting_message_t *greetmsg = (FUSE_greeting_message_t *)malloc(sizeof(FUSE_greeting_message_t));
  greetmsg->version = htonl(FUSE_CURRENT_VERSION);
  __outbound_msgq->push(new FuseNetworkMessage(FUSE_MT_GREETING,
					       greetmsg, sizeof(FUSE_greeting_message_t)));
}


/** Disconnect. */
void
FuseClient::disconnect()
{
  __mutex->lock();
  delete __socket;
  __socket = new StreamSocket();
  __alive = false;
  __mutex->unlock();
}


/** Send queued messages. */
void
FuseClient::send()
{
  try {
    FuseNetworkTransceiver::send(__socket, __outbound_msgq);
  } catch (ConnectionDiedException &e) {
    e.print_trace();
    __socket->close();
    __alive = false;
    throw;
  }
}


/** Receive messages. */
void
FuseClient::recv()
{
  try {
    while ( __socket->available() ) {
      FuseNetworkTransceiver::recv(__socket, __inbound_msgq);
    }
  } catch (ConnectionDiedException &e) {
    e.print_trace();
    __socket->close();
    __alive = false;
    throw;
  }
}


/** Enqueue message.
 * @param m message to enqueue
 */
void
FuseClient::enqueue(FuseNetworkMessage *m)
{
  m->ref();
  __outbound_msgq->push_locked(m);
}


/** Enqueue message.
 * @param type type of message
 * @param payload payload of message
 * @param payload_size size of payload
 */
void
FuseClient::enqueue(FUSE_message_type_t type, void *payload, size_t payload_size)
{
  FuseNetworkMessage *m = new FuseNetworkMessage(type, payload, payload_size);
  __outbound_msgq->push_locked(m);  
}


/** Enqueue message without payload.
 * @param type type of message
 */
void
FuseClient::enqueue(FUSE_message_type_t type)
{
  FuseNetworkMessage *m = new FuseNetworkMessage(type);
  __outbound_msgq->push_locked(m);  
}


/** Sleep for some time.
 * Wait until inbound messages have been receive, the connection dies or the
 * timeout has been reached, whatever comes first. So you sleep at most timeout ms,
 * but short under some circumstances (incoming data or lost connection).
 */
void
FuseClient::sleep()
{
  try {
    __socket->poll(__wait_timeout /* ms timeout */, Socket::POLL_IN);
  } catch (Exception &e) {
  }
}


/** Thread loop.
 * Sends enqueued messages and reads incoming messages off the network.
 */
void
FuseClient::loop()
{
  __mutex->lock();

  if ( ! __alive ) {
    usleep(10000);
    return;
  }

  bool wake = false;

  send();
  sleep();
  recv();

  //process_inbound();

  __inbound_msgq->lock();
  while ( ! __inbound_msgq->empty() ) {
    FuseNetworkMessage *m = __inbound_msgq->front();

    wake = true;

    if ( m->type() == FUSE_MT_GREETING ) {
      FUSE_greeting_message_t *gm = m->msg<FUSE_greeting_message_t>();
      if ( ntohl(gm->version) != FUSE_CURRENT_VERSION ) {
	__handler->fuse_invalid_server_version(FUSE_CURRENT_VERSION, ntohl(gm->version));
	__alive = false;
      } else {
	__handler->fuse_connection_established();
      }
    } else {
      __handler->fuse_inbound_received(m);
    }

    m->unref();
    __inbound_msgq->pop();
  }
  __inbound_msgq->unlock();

  if ( wake ) {
    __waitcond->wake_all();
  }
  __mutex->unlock();
}


/** Wait for messages.
 * This will wait for messages of to arrive. The calling
 * thread is blocked until messages are available.
 */
void
FuseClient::wait()
{
  Mutex m;
  m.lock();
  __waitcond->wait(&m);
}
