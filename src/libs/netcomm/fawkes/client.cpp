
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
#include <core/threading/mutex_locker.h>
#include <core/threading/wait_condition.h>

#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/client_handler.h>
#include <netcomm/fawkes/message_queue.h>
#include <netcomm/fawkes/transceiver.h>
#include <netcomm/socket/stream.h>
#include <netcomm/utils/exceptions.h>

#include <list>

/** @class HandlerAlreadyRegisteredException netcomm/fawkes/client.h
 * Client handler has already been registered.
 * Only a single client handler can be registered per component. If you try
 * to register a handler where there is already a handler this exception
 * is thrown.
 * @ingroup NetComm
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
 * @ingroup NetComm
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
  this->hostname = strdup(hostname);
  this->port     = port;

  wait_timeout = 10;

  mutex = new Mutex();
  s = NULL;
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
  free(hostname);
}


/** Connect to remote.
 * @exception SocketException thrown by Socket::connect()
 */
void
FawkesNetworkClient::connect()
{
  MutexLocker lock(mutex);
  try {
    s = new StreamSocket();
    s->connect(hostname, port);
    notify_of_connection_established();
  } catch (SocketException &e) {
    delete s;
    s = NULL;
    throw;
  }
}


/** Disconnect socket. */
void
FawkesNetworkClient::disconnect()
{
  mutex->lock();
  delete s;
  s = NULL;
  notify_of_connection_dead();
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
  FawkesNetworkTransceiver::recv(s, inbound_msgq);
}


/** Set the timeout for incoming data.
 * The thread will poll on the socket for the given time for incoming
 * data. If no data is received it will look at the outbound queue and send
 * all enqueued data and again wait for incoming data.
 * @param wait_timeout new timeout in miliseconds
 */
void
FawkesNetworkClient::set_wait_timeout(unsigned int wait_timeout)
{
  this->wait_timeout = wait_timeout;
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
  try {
    s->poll(wait_timeout /* ms timeout */, Socket::POLL_IN);
  } catch (Exception &e) {
    // ignored
  }
}


/** Register handler.
 * Handlers are used to handle incoming packets. There may only be one handler per
 * component!
 * @param handler handler to register
 * @param component_id component ID to register the handler for.
 */
void
FawkesNetworkClient::register_handler(FawkesNetworkClientHandler *handler, unsigned int component_id)
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
FawkesNetworkClient::deregister_handler(unsigned int component_id)
{
  mutex->lock();
  if ( handlers.find(component_id) != handlers.end() ) {
    handlers[component_id]->deregistered();
    handlers.erase(component_id);
  }
  if ( waitconds.find(component_id) != waitconds.end() ) {
    WaitCondition *wc = waitconds[component_id];
    waitconds.erase(component_id);
    wc->wake_all();
    delete wc;
  }
  mutex->unlock();
}


void
FawkesNetworkClient::notify_of_connection_dead()
{
  for ( HandlerMap::iterator i = handlers.begin(); i != handlers.end(); ++i ) {
    (*i).second->connection_died();
  }
  for ( WaitCondMap::iterator j = waitconds.begin(); j != waitconds.end(); ++j) {
    (*j).second->wake_all();
  }
}

void
FawkesNetworkClient::notify_of_connection_established()
{
  for ( HandlerMap::iterator i = handlers.begin(); i != handlers.end(); ++i ) {
    (*i).second->connection_established();
  }
  for ( WaitCondMap::iterator j = waitconds.begin(); j != waitconds.end(); ++j) {
    (*j).second->wake_all();
  }
}

/** Thread loop.
 * Sends enqueued messages and reads incoming messages off the network.
 */
void
FawkesNetworkClient::loop()
{
  MutexLocker lock(mutex);
  std::list<unsigned int> wakeup_list;

  // just return if not connected
  if (! s ) return;

  try {
    send();
    sleep();
    recv();
  } catch (ConnectionDiedException &cde) {
    delete s;
    s = NULL;
    notify_of_connection_dead();
  }

  inbound_msgq->lock();
  while ( ! inbound_msgq->empty() ) {
    FawkesNetworkMessage *m = inbound_msgq->front();
    unsigned int cid = m->cid();
    if (handlers.find(cid) != handlers.end()) {
      handlers[cid]->inbound_received(m);
      wakeup_list.push_back(cid);
    }
    m->unref();
    inbound_msgq->pop();
  }
  inbound_msgq->unlock();

  wakeup_list.sort();
  wakeup_list.unique();
  for (std::list<unsigned int>::iterator i = wakeup_list.begin(); i != wakeup_list.end(); ++i) {
    waitconds[*i]->wake_all();
  }
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
    waitconds[component_id]->wait();
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
    waitconds[component_id]->wake_all();
  }
}


/** Check if connection is alive.
 * @return true if connection is alive at the moment, false otherwise
 */
bool
FawkesNetworkClient::connected() const throw()
{
  return (s != NULL);
}
