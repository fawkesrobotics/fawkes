
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

#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>

#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/client_handler.h>
#include <netcomm/fawkes/message_queue.h>
#include <netcomm/fawkes/transceiver.h>
#include <netcomm/socket/stream.h>

#include <list>

/** @class HandlerAlreadyRegisteredException netcomm/fawkes/client.h
 * Client handler has already been registered.
 * Only a single client handler can be registered per component. If you try
 * to register a handler where there is already a handler this exception
 * is thrown.
 */

/** Costructor. */
HandlerAlreadyRegisteredException::HandlerAlreadyRegisteredException()
  : Exception("A handler for this component has already been registered")
{
}


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
  : Thread("FawkesNetworkClient")
{
  inbound_msgq = new FawkesNetworkMessageQueue();
  outbound_msgq = new FawkesNetworkMessageQueue();
  s = new StreamSocket();
  this->hostname = hostname;
  this->port     = port;

  wait_timeout = 10;

  mutex = new Mutex();
}


/** Destructor. */
FawkesNetworkClient::~FawkesNetworkClient()
{
  inbound_msgq->lock();
  while ( ! inbound_msgq->empty() ) {
    FawkesNetworkMessage *m = inbound_msgq->front();
    m->unref();
    inbound_msgq->pop();
  }
  inbound_msgq->unlock();  
  delete inbound_msgq;

  outbound_msgq->lock();
  while ( ! outbound_msgq->empty() ) {
    FawkesNetworkMessage *m = outbound_msgq->front();
    m->unref();
    outbound_msgq->pop();
  }
  outbound_msgq->unlock();  
  delete outbound_msgq;

  for (std::map<unsigned int, WaitCondition *>::iterator i =  waitconds.begin(); i != waitconds.end(); ++i ) {
    delete (*i).second;
  }
  waitconds.clear();
  delete s;
  delete mutex;
}


/** Connect to remote.
 * @exception SocketException thrown by Socket::connect()
 */
void
FawkesNetworkClient::connect()
{
  mutex->lock();
  try {
    s->connect(hostname, port);
  } catch (SocketException &e) {
    mutex->unlock();
    throw;
  }
  mutex->unlock();
}


/** Disconnect socket. */
void
FawkesNetworkClient::disconnect()
{
  mutex->lock();
  delete s;
  s = new StreamSocket();
  mutex->unlock();
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
  FawkesNetworkTransceiver::send(s, outbound_msgq);
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
FawkesNetworkClient::setNoDelay(bool nodelay)
{
  s->set_nodelay(nodelay);
}


/** Set the timeout for incoming data.
 * The thread will poll on the socket for the given time for incoming
 * data. If no data is received it will look at the outbound queue and send
 * all enqueued data and again wait for incoming data.
 * @param wait_timeout new timeout in miliseconds
 */
void
FawkesNetworkClient::setWaitTimeout(unsigned int wait_timeout)
{
  this->wait_timeout = wait_timeout;
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


/** Sleep for some time.
 * Wait until inbound messages have been receive, the connection dies or the
 * timeout has been reached, whatever comes first. So you sleep at most timeout ms,
 * but short under some circumstances (incoming data or lost connection).
 * @see setWaitTimeout()
 */
void
FawkesNetworkClient::sleep()
{
  short p = 0;
  try {
    p = s->poll(wait_timeout /* ms timeout */, Socket::POLL_IN);
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
}


/** Register handler.
 * Handlers are used to handle incoming packets. There may only be one handler per
 * component!
 * @param handler handler to register
 * @param component_id component ID to register the handler for.
 */
void
FawkesNetworkClient::registerHandler(FawkesNetworkClientHandler *handler, unsigned int component_id)
{
  mutex->lock();
  if ( handlers.find(component_id) != handlers.end() ) {
    mutex->unlock();
    throw HandlerAlreadyRegisteredException();
  } else {
    handlers[component_id] = handler;
    waitconds[component_id] = new WaitCondition();
  }
  mutex->unlock();
}


/** Deregister handler.
 * @param component_id component ID
 */
void
FawkesNetworkClient::deregisterHandler(unsigned int component_id)
{
  mutex->lock();
  if ( handlers.find(component_id) != handlers.end() ) {
    handlers[component_id]->deregistered();
    handlers.erase(component_id);
  }
  if ( waitconds.find(component_id) != waitconds.end() ) {
    delete waitconds[component_id];
    waitconds.erase(component_id);
  }
  mutex->unlock();
}


/** Thread loop.
 * Sends enqueued messages and reads incoming messages off the network.
 */
void
FawkesNetworkClient::loop()
{
  std::list<unsigned int> wakeup_list;

  mutex->lock();

  send();
  sleep();
  recv();

  inbound_msgq->lock();
  while ( ! inbound_msgq->empty() ) {
    FawkesNetworkMessage *m = inbound_msgq->front();
    unsigned int cid = m->cid();
    if (handlers.find(cid) != handlers.end()) {
      handlers[cid]->inboundReceived(m);
      wakeup_list.push_back(cid);
    }
    m->unref();
    inbound_msgq->pop();
  }
  inbound_msgq->unlock();

  wakeup_list.sort();
  wakeup_list.unique();
  for (std::list<unsigned int>::iterator i = wakeup_list.begin(); i != wakeup_list.end(); ++i) {
    waitconds[*i]->wakeAll();
  }

  mutex->unlock();
}


/** Wait for messages for component ID.
 * This will wait for messages of the given component ID to arrive. The calling
 * thread is blocked until messages are available.
 * @param component_id component ID to monitor
 */
void
FawkesNetworkClient::wait(unsigned int component_id)
{
  if ( waitconds.find(component_id) != waitconds.end() ) {
    Mutex m;
    m.lock();
    waitconds[component_id]->wait(&m);
    m.unlock();
  }
}

/** Wake a waiting thread.
 * This will wakeup all threads currently waiting for the specified component ID.
 * This can be helpful to wake a sleeping thread if you received a signal.
 * @param component_id component ID for threads to wake up
 */
void
FawkesNetworkClient::wake(unsigned int component_id)
{
  if ( waitconds.find(component_id) != waitconds.end() ) {
    waitconds[component_id]->wakeAll();
  }
}
