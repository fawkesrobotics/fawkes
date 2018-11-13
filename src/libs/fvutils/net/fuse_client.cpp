
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
  hostname_ = strdup(hostname);
  port_ = port;
  handler_ = handler;

  wait_timeout_ = 10;

  inbound_msgq_ = new FuseNetworkMessageQueue();
  outbound_msgq_ = new FuseNetworkMessageQueue();

  mutex_ = new Mutex();
  recv_mutex_    = new Mutex();
  recv_waitcond_ = new WaitCondition(recv_mutex_);
  socket_ = new StreamSocket();
  greeting_mutex_    = new Mutex();
  greeting_waitcond_ = new WaitCondition(greeting_mutex_);

  alive_ = true;
  greeting_received_ = false;
}


/** Destructor. */
FuseClient::~FuseClient()
{
  free(hostname_);

  while ( ! inbound_msgq_->empty() ) {
    FuseNetworkMessage *m = inbound_msgq_->front();
    m->unref();
    inbound_msgq_->pop();
  }
  delete inbound_msgq_;

  while ( ! outbound_msgq_->empty() ) {
    FuseNetworkMessage *m = outbound_msgq_->front();
    m->unref();
    outbound_msgq_->pop();
  }
  delete outbound_msgq_;

  delete mutex_;
  delete recv_mutex_;
  delete recv_waitcond_;
  delete socket_;
  delete greeting_mutex_;
  delete greeting_waitcond_;
}


/** Connect. */
void
FuseClient::connect()
{
  socket_->connect(hostname_, port_);

  FUSE_greeting_message_t *greetmsg = (FUSE_greeting_message_t *)malloc(sizeof(FUSE_greeting_message_t));
  greetmsg->version = htonl(FUSE_CURRENT_VERSION);
  outbound_msgq_->push(new FuseNetworkMessage(FUSE_MT_GREETING,
					       greetmsg, sizeof(FUSE_greeting_message_t)));
}


/** Disconnect. */
void
FuseClient::disconnect()
{
  mutex_->lock();
  delete socket_;
  socket_ = new StreamSocket();
  alive_ = false;
  mutex_->unlock();
}


/** Send queued messages. */
void
FuseClient::send()
{
  try {
    FuseNetworkTransceiver::send(socket_, outbound_msgq_);
  } catch (ConnectionDiedException &e) {
    e.print_trace();
    socket_->close();
    alive_ = false;
    handler_->fuse_connection_died();
    recv_waitcond_->wake_all();
  }
}


/** Receive messages. */
void
FuseClient::recv()
{
  recv_mutex_->lock();
  try {
    while ( socket_->available() ) {
      FuseNetworkTransceiver::recv(socket_, inbound_msgq_);
    }
  } catch (ConnectionDiedException &e) {
    e.print_trace();
    socket_->close();
    alive_ = false;
    handler_->fuse_connection_died();
    recv_waitcond_->wake_all();
  }
  recv_mutex_->unlock();
}


/** Enqueue message.
 * This method takes ownership of the passed message. You must explicitly
 * reference it before enqueing if you want to use it afterwards.
 * @param m message to enqueue
 */
void
FuseClient::enqueue(FuseNetworkMessage *m)
{
  outbound_msgq_->push_locked(m);
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
  outbound_msgq_->push_locked(m);  
}


/** Enqueue message without payload.
 * @param type type of message
 */
void
FuseClient::enqueue(FUSE_message_type_t type)
{
  FuseNetworkMessage *m = new FuseNetworkMessage(type);
  outbound_msgq_->push_locked(m);  
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
  recv_mutex_->lock();
  outbound_msgq_->push_locked(m);
  recv_waitcond_->wait();
  recv_mutex_->unlock();
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
  recv_mutex_->lock();
  outbound_msgq_->push_locked(m);  
  recv_waitcond_->wait();
  recv_mutex_->unlock();
}


/** Enqueue message without payload and wait for reply.
 * The wait happens atomically, use this to avoid race conditions.
 * @param type type of message
 */
void
FuseClient::enqueue_and_wait(FUSE_message_type_t type)
{
  FuseNetworkMessage *m = new FuseNetworkMessage(type);
  recv_mutex_->lock();
  outbound_msgq_->push_locked(m);  
  recv_waitcond_->wait();
  recv_mutex_->unlock();
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
    socket_->poll(wait_timeout_ /* ms timeout */, Socket::POLL_IN);
  } catch (Exception &e) {
  }
}


/** Thread loop.
 * Sends enqueued messages and reads incoming messages off the network.
 */
void
FuseClient::loop()
{
  mutex_->lock();

  if ( ! alive_ ) {
    mutex_->unlock();
    usleep(10000);
    return;
  }

  bool wake = false;

  send();
  sleep();
  recv();

  //process_inbound();

  inbound_msgq_->lock();
  while ( ! inbound_msgq_->empty() ) {
    FuseNetworkMessage *m = inbound_msgq_->front();

    if ( m->type() == FUSE_MT_GREETING ) {
      FUSE_greeting_message_t *gm = m->msg<FUSE_greeting_message_t>();
      if ( ntohl(gm->version) != FUSE_CURRENT_VERSION ) {
	handler_->fuse_invalid_server_version(FUSE_CURRENT_VERSION, ntohl(gm->version));
	alive_ = false;
      } else {
	greeting_mutex_->lock();
	greeting_received_ = true;
	greeting_waitcond_->wake_all();
	greeting_mutex_->unlock();
	handler_->fuse_connection_established();
      }
    } else {
      handler_->fuse_inbound_received(m);
      wake = true;
    }

    m->unref();
    inbound_msgq_->pop();
  }
  inbound_msgq_->unlock();

  if ( wake ) {
    recv_waitcond_->wake_all();
  }
  mutex_->unlock();
}


/** Wait for messages.
 * This will wait for messages to arrive. The calling
 * thread is blocked until messages are available.
 */
void
FuseClient::wait()
{
  recv_mutex_->lock();
  recv_waitcond_->wait();
  recv_mutex_->unlock();
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
  greeting_mutex_->lock();
  while (! greeting_received_) {
    greeting_waitcond_->wait();
  }
  greeting_mutex_->unlock();
}

} // end namespace firevision
