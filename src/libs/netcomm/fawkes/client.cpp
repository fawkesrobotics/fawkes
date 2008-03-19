
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
#include <netcomm/fawkes/client_handler.h>
#include <netcomm/fawkes/message_queue.h>
#include <netcomm/fawkes/transceiver.h>
#include <netcomm/socket/stream.h>
#include <netcomm/utils/exceptions.h>

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>
#include <core/exceptions/system.h>

#include <list>
#include <cstring>
#include <cstdlib>
#include <unistd.h>


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


/** Fawkes network client send thread.
 * Spawned by the FawkesNetworkClient to handle outgoing traffic.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */
class FawkesNetworkClientSendThread : public Thread
{
 public:

  /** Constructor.
   * @param s client stream socket
   * @param parent parent FawkesNetworkClient instance
   */
  FawkesNetworkClientSendThread(StreamSocket *s, FawkesNetworkClient *parent)
    : Thread("FawkesNetworkClientSendThread", Thread::OPMODE_WAITFORWAKEUP)
  {
    _s = s;
    _parent = parent;
    _outbound_msgq = new FawkesNetworkMessageQueue();
  }

  /** Destructor. */
  ~FawkesNetworkClientSendThread()
  {
    while ( ! _outbound_msgq->empty() ) {
      FawkesNetworkMessage *m = _outbound_msgq->front();
      m->unref();
      _outbound_msgq->pop();
    }
    delete _outbound_msgq;
  }

  virtual void once()
  {
    _parent->set_send_slave_alive();
  }

  virtual void loop()
  {
    if ( ! _parent->connected() )  return;

    if ( ! _outbound_msgq->empty() ) {
      try {
	FawkesNetworkTransceiver::send(_s, _outbound_msgq);
      } catch (ConnectionDiedException &e) {
	_parent->connection_died();
	exit();
      }
    }
  }

  /** Force sending of messages.
   * All messages are sent out immediately, if loop is not running already anyway.
   */
  void force_send()
  {
    if ( loop_mutex->try_lock() ) {
      loop();
      loop_mutex->unlock();
    }
  }

  /** Enqueue message to send.
   * @param message message to send
   */
  void enqueue(FawkesNetworkMessage *message)
  {
    message->ref();
    _outbound_msgq->push_locked(message);
    wakeup();
  }

 private:
  StreamSocket *_s;
  FawkesNetworkClient *_parent;
  FawkesNetworkMessageQueue *_outbound_msgq;
};


/**  Fawkes network client receive thread.
 * Spawned by the FawkesNetworkClient to handle incoming traffic.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */
class FawkesNetworkClientRecvThread : public Thread
{
 public:
  /** Constructor.
   * @param s client stream socket
   * @param parent parent FawkesNetworkClient instance
   */
  FawkesNetworkClientRecvThread(StreamSocket *s, FawkesNetworkClient *parent)
    : Thread("FawkesNetworkClientRecvThread")
  {
    _s = s;
    _parent = parent;
    inbound_msgq = new FawkesNetworkMessageQueue();
  }

  /** Destructor. */
  ~FawkesNetworkClientRecvThread()
  {
    while ( ! inbound_msgq->empty() ) {
      FawkesNetworkMessage *m = inbound_msgq->front();
      m->unref();
      inbound_msgq->pop();
    }
    delete inbound_msgq;
  }

  /** Receive and process messages. */
  void recv()
  {
    std::list<unsigned int> wakeup_list;

    try {
      FawkesNetworkTransceiver::recv(_s, inbound_msgq);

      inbound_msgq->lock();
      while ( ! inbound_msgq->empty() ) {
	FawkesNetworkMessage *m = inbound_msgq->front();
	wakeup_list.push_back(m->cid());
	_parent->dispatch_message(m);
	m->unref();
	inbound_msgq->pop();
      }
      inbound_msgq->unlock();
    
      wakeup_list.sort();
      wakeup_list.unique();
      for (std::list<unsigned int>::iterator i = wakeup_list.begin(); i != wakeup_list.end(); ++i) {
	_parent->wake_handlers(*i);
      }
    } catch (ConnectionDiedException &e) {
      throw;
    }
  }


  virtual void once()
  {
    _parent->set_recv_slave_alive();
  }

  virtual void loop()
  {
    // just return if not connected
    if (! _s ) return;

    short p = 0;
    try {
      p = _s->poll();
    } catch (InterruptedException &e) {
      return;
    }

    if ( (p & Socket::POLL_ERR) ||
	 (p & Socket::POLL_HUP) ||
	 (p & Socket::POLL_RDHUP)) {
      _parent->connection_died();
      exit();
    } else if ( p & Socket::POLL_IN ) {
      // Data can be read
      try {
	recv();
      } catch (ConnectionDiedException &e) {
	_parent->connection_died();
	exit();
      }
    }
  }

 private:
  StreamSocket *_s;
  FawkesNetworkClient *_parent;
  FawkesNetworkMessageQueue *  inbound_msgq;
};


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
{
  this->hostname = strdup(hostname);
  this->port     = port;

  s = NULL;
  send_slave = NULL;
  recv_slave = NULL;

  connection_died_recently = false;
  send_slave_alive = false;
  recv_slave_alive = false;

  slave_status_mutex = new Mutex();

  _id     = 0;
  _has_id = false;
}


/** Constructor.
 * @param id id of the client.
 * @param hostname remote host to connect to.
 * @param port port to connect to.
 */
FawkesNetworkClient::FawkesNetworkClient(unsigned int id, const char *hostname,
					 unsigned short int port)
{
  this->hostname = strdup(hostname);
  this->port     = port;

  s = NULL;
  send_slave = NULL;
  recv_slave = NULL;

  connection_died_recently = false;
  send_slave_alive = false;
  recv_slave_alive = false;

  slave_status_mutex = new Mutex();

  _id     = id;
  _has_id = true;
}


/** Destructor. */
FawkesNetworkClient::~FawkesNetworkClient()
{
  disconnect();

  for (std::map<unsigned int, WaitCondition *>::iterator i =  waitconds.begin(); i != waitconds.end(); ++i ) {
    delete (*i).second;
  }
  waitconds.clear();
  delete s;
  free(hostname);
  delete slave_status_mutex;
}


/** Connect to remote.
 * @exception SocketException thrown by Socket::connect()
 */
void
FawkesNetworkClient::connect()
{
  try {
    s = new StreamSocket();
    s->connect(hostname, port);
    send_slave = new FawkesNetworkClientSendThread(s, this);
    send_slave->start();
    recv_slave = new FawkesNetworkClientRecvThread(s, this);
    recv_slave->start();
    connection_died_recently = false;
  } catch (SocketException &e) {
    if ( send_slave ) {
      send_slave->cancel();
      send_slave->join();
      delete send_slave;
      send_slave = NULL;
    }
    if ( recv_slave ) {
      recv_slave->cancel();
      recv_slave->join();
      delete recv_slave;
      recv_slave = NULL;
    }
    send_slave_alive = false;
    recv_slave_alive = false;
    delete s;
    s = NULL;
    throw;
  }
}


/** Disconnect socket. */
void
FawkesNetworkClient::disconnect()
{
  if ( s == NULL ) return;

  if ( send_slave_alive ) {
    if ( ! connection_died_recently ) {
      send_slave->force_send();
      // Give other side some time to read the messages just sent
      usleep(100000);
    }
    send_slave->cancel();
    send_slave->join();
    delete send_slave;
    send_slave = NULL;
  }
  if ( recv_slave_alive ) {
    recv_slave->cancel();
    recv_slave->join();
    delete recv_slave;
    recv_slave = NULL;
  }
  send_slave_alive = false;
  recv_slave_alive = false;
  delete s;
  s = NULL;
}


/** Enqueue message to send.
 * @param message message to send
 */
void
FawkesNetworkClient::enqueue(FawkesNetworkMessage *message)
{
  if (send_slave)  send_slave->enqueue(message);
}


/** Register handler.
 * Handlers are used to handle incoming packets. There may only be one handler per
 * component!
 * @param handler handler to register
 * @param component_id component ID to register the handler for.
 */
void
FawkesNetworkClient::register_handler(FawkesNetworkClientHandler *handler,
				      unsigned int component_id)
{
  handlers.lock();
  if ( handlers.find(component_id) != handlers.end() ) {
    handlers.unlock();
    throw HandlerAlreadyRegisteredException();
  } else {
    handlers[component_id] = handler;
    waitconds[component_id] = new WaitCondition();
  }
  handlers.unlock();
}


/** Deregister handler.
 * @param component_id component ID
 */
void
FawkesNetworkClient::deregister_handler(unsigned int component_id)
{
  handlers.lock();
  if ( handlers.find(component_id) != handlers.end() ) {
    handlers[component_id]->deregistered(_id);
    handlers.erase(component_id);
  }
  if ( waitconds.find(component_id) != waitconds.end() ) {
    WaitCondition *wc = waitconds[component_id];
    waitconds.erase(component_id);
    wc->wake_all();
    delete wc;
  }
  handlers.unlock();
}


void
FawkesNetworkClient::dispatch_message(FawkesNetworkMessage *m)
{
  unsigned int cid = m->cid();
  if (handlers.find(cid) != handlers.end()) {
    handlers[cid]->inbound_received(m, _id);
  }
}


void
FawkesNetworkClient::wake_handlers(unsigned int cid)
{
  if ( waitconds.find(cid) != waitconds.end() ) {
    waitconds[cid]->wake_all();
  }
}

void
FawkesNetworkClient::notify_of_connection_dead()
{
  for ( HandlerMap::iterator i = handlers.begin(); i != handlers.end(); ++i ) {
    (*i).second->connection_died(_id);
  }
  for ( WaitCondMap::iterator j = waitconds.begin(); j != waitconds.end(); ++j) {
    (*j).second->wake_all();
  }
}

void
FawkesNetworkClient::notify_of_connection_established()
{
  for ( HandlerMap::iterator i = handlers.begin(); i != handlers.end(); ++i ) {
    (*i).second->connection_established(_id);
  }
  for ( WaitCondMap::iterator j = waitconds.begin(); j != waitconds.end(); ++j) {
    (*j).second->wake_all();
  }
}


void
FawkesNetworkClient::connection_died()
{
  connection_died_recently = true;
  notify_of_connection_dead();
}


void
FawkesNetworkClient::set_send_slave_alive()
{
  slave_status_mutex->lock();
  send_slave_alive = true;
  if ( send_slave_alive && recv_slave_alive ) {
    notify_of_connection_established();
  }
  slave_status_mutex->unlock();
}


void
FawkesNetworkClient::set_recv_slave_alive()
{
  slave_status_mutex->lock();
  recv_slave_alive = true;
  if ( send_slave_alive && recv_slave_alive ) {
    notify_of_connection_established();
  }
  slave_status_mutex->unlock();
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
  return (! connection_died_recently && (s != NULL));
}


/** Check whether the client has an id.
 * @return true if client has an ID
 */
bool
FawkesNetworkClient::has_id() const
{
  return _has_id;
}


/** Get the client's ID.
 * @return the ID
 */
unsigned int
FawkesNetworkClient::id() const
{
  if ( !_has_id ) {
    throw Exception("Trying to get the ID of a client that has no ID");
  }

  return _id;
}
