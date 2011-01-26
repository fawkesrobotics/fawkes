
/***************************************************************************
 *  fuse_client.cpp - FUSE network transport client
 *
 *  Created: Thu Mar 29 00:47:24 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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
#include <cstdlib>
#include <unistd.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

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
  __recv_mutex    = new Mutex();
  __recv_waitcond = new WaitCondition(__recv_mutex);
  __socket = new StreamSocket();
  __greeting_mutex    = new Mutex();
  __greeting_waitcond = new WaitCondition(__greeting_mutex);

  __alive = true;
  __greeting_received = false;
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
  delete __recv_mutex;
  delete __recv_waitcond;
  delete __socket;
  delete __greeting_mutex;
  delete __greeting_waitcond;
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
    __handler->fuse_connection_died();
    __recv_waitcond->wake_all();
  }
}


/** Receive messages. */
void
FuseClient::recv()
{
  __recv_mutex->lock();
  try {
    while ( __socket->available() ) {
      FuseNetworkTransceiver::recv(__socket, __inbound_msgq);
    }
  } catch (ConnectionDiedException &e) {
    e.print_trace();
    __socket->close();
    __alive = false;
    __handler->fuse_connection_died();
    __recv_waitcond->wake_all();
  }
  __recv_mutex->unlock();
}


/** Enqueue message.
 * This method takes ownership of the passed message. You must explicitly
 * reference it before enqueing if you want to use it afterwards.
 * @param m message to enqueue
 */
void
FuseClient::enqueue(FuseNetworkMessage *m)
{
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


/** Enqueue message and wait for reply.
 * The wait happens atomically, use this to avoid race conditions. This method
 * takes ownership of the passed message. You must explicitly reference it
 * before enqueing if you want to use it afterwards.
 * @param m message to enqueue
 */
void
FuseClient::enqueue_and_wait(FuseNetworkMessage *m)
{
  __recv_mutex->lock();
  __outbound_msgq->push_locked(m);
  __recv_waitcond->wait();
  __recv_mutex->unlock();
}


/** Enqueue message and wait for reply.
 * The wait happens atomically, use this to avoid race conditions.
 * @param type type of message
 * @param payload payload of message
 * @param payload_size size of payload
 */
void
FuseClient::enqueue_and_wait(FUSE_message_type_t type, void *payload, size_t payload_size)
{
  FuseNetworkMessage *m = new FuseNetworkMessage(type, payload, payload_size);
  __recv_mutex->lock();
  __outbound_msgq->push_locked(m);  
  __recv_waitcond->wait();
  __recv_mutex->unlock();
}


/** Enqueue message without payload and wait for reply.
 * The wait happens atomically, use this to avoid race conditions.
 * @param type type of message
 */
void
FuseClient::enqueue_and_wait(FUSE_message_type_t type)
{
  FuseNetworkMessage *m = new FuseNetworkMessage(type);
  __recv_mutex->lock();
  __outbound_msgq->push_locked(m);  
  __recv_waitcond->wait();
  __recv_mutex->unlock();
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
    __mutex->unlock();
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

    if ( m->type() == FUSE_MT_GREETING ) {
      FUSE_greeting_message_t *gm = m->msg<FUSE_greeting_message_t>();
      if ( ntohl(gm->version) != FUSE_CURRENT_VERSION ) {
	__handler->fuse_invalid_server_version(FUSE_CURRENT_VERSION, ntohl(gm->version));
	__alive = false;
      } else {
	__greeting_mutex->lock();
	__greeting_received = true;
	__greeting_waitcond->wake_all();
	__greeting_mutex->unlock();
	__handler->fuse_connection_established();
      }
    } else {
      __handler->fuse_inbound_received(m);
      wake = true;
    }

    m->unref();
    __inbound_msgq->pop();
  }
  __inbound_msgq->unlock();

  if ( wake ) {
    __recv_waitcond->wake_all();
  }
  __mutex->unlock();
}


/** Wait for messages.
 * This will wait for messages to arrive. The calling
 * thread is blocked until messages are available.
 */
void
FuseClient::wait()
{
  __recv_mutex->lock();
  __recv_waitcond->wait();
  __recv_mutex->unlock();
}


/** Wait for greeting message.
 * This method will wait for the greeting message to arrive. Make sure that you called
 * connect() before waiting or call it concurrently in another thread. The calling thread
 * will be blocked until the message has been received. If the message has already been
 * received this method will return immediately. Thus it is safe to call this at any time
 * without risking a race condition.
 */
void
FuseClient::wait_greeting()
{
  __greeting_mutex->lock();
  while (! __greeting_received) {
    __greeting_waitcond->wait();
  }
  __greeting_mutex->unlock();
}

} // end namespace firevision
