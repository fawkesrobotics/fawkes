
/***************************************************************************
 *  client.cpp - Fawkes network client
 *
 *  Created: Tue Nov 21 18:44:58 2006
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/client_handler.h>
#include <netcomm/fawkes/message_queue.h>
#include <netcomm/fawkes/transceiver.h>
#include <netcomm/socket/stream.h>
#include <netcomm/utils/exceptions.h>

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/wait_condition.h>
#include <core/exceptions/system.h>

#include <list>
#include <cstring>
#include <cstdlib>
#include <unistd.h>

namespace fawkes {

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
    __s = s;
    __parent = parent;
    __outbound_mutex    = new Mutex();
    __outbound_msgqs[0] = new FawkesNetworkMessageQueue();
    __outbound_msgqs[1] = new FawkesNetworkMessageQueue();
    __outbound_active   = 0;
    __outbound_msgq     = __outbound_msgqs[0];
    __outbound_havemore = false;
  }

  /** Destructor. */
  ~FawkesNetworkClientSendThread()
  {
    for (unsigned int i = 0; i < 2; ++i) {
      while ( ! __outbound_msgqs[i]->empty() ) {
	FawkesNetworkMessage *m = __outbound_msgqs[i]->front();
	m->unref();
	__outbound_msgqs[i]->pop();
      }
    }
    delete __outbound_msgqs[0];
    delete __outbound_msgqs[1];
    delete __outbound_mutex;
  }

  virtual void once()
  {
    __parent->set_send_slave_alive();
  }

  virtual void loop()
  {
    if ( ! __parent->connected() )  return;

    while ( __outbound_havemore ) {
      __outbound_mutex->lock();
      __outbound_havemore = false;
      FawkesNetworkMessageQueue *q = __outbound_msgq;
      __outbound_active = 1 - __outbound_active;
      __outbound_msgq = __outbound_msgqs[__outbound_active];
      __outbound_mutex->unlock();

      if ( ! q->empty() ) {
	try {
	  FawkesNetworkTransceiver::send(__s, q);
	} catch (ConnectionDiedException &e) {
	  __parent->connection_died();
	  exit();
	}
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

  /** Enqueue message to send and take ownership.
   * This method takes ownership of the message. If you want to use the message
   * after enqueing you must reference:
   * @code
   * message->ref();
   * send_slave->enqueue(message);
   * // message can now still be used
   * @endcode
   * Without extra referencing the message may not be used after enqueuing.
   * @param message message to send
   */
  void enqueue(FawkesNetworkMessage *message)
  {
    __outbound_mutex->lock();
    __outbound_msgq->push(message);
    __outbound_havemore = true;
    __outbound_mutex->unlock();
    wakeup();
  }

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  StreamSocket *__s;
  FawkesNetworkClient *__parent;
  Mutex                     *__outbound_mutex;
  unsigned int               __outbound_active;
  bool                       __outbound_havemore;
  FawkesNetworkMessageQueue *__outbound_msgq;
  FawkesNetworkMessageQueue *__outbound_msgqs[2];
  
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
   * @param recv_mutex receive mutex, locked while messages are received
   */
  FawkesNetworkClientRecvThread(StreamSocket *s, FawkesNetworkClient *parent,
				Mutex *recv_mutex)
    : Thread("FawkesNetworkClientRecvThread")
  {
    __s = s;
    __parent = parent;
    __inbound_msgq = new FawkesNetworkMessageQueue();
    __recv_mutex = recv_mutex;
  }

  /** Destructor. */
  ~FawkesNetworkClientRecvThread()
  {
    while ( ! __inbound_msgq->empty() ) {
      FawkesNetworkMessage *m = __inbound_msgq->front();
      m->unref();
      __inbound_msgq->pop();
    }
    delete __inbound_msgq;
  }

  /** Receive and process messages. */
  void recv()
	{
    std::list<unsigned int> wakeup_list;

    try {
      FawkesNetworkTransceiver::recv(__s, __inbound_msgq);

      MutexLocker lock(__recv_mutex);

      __inbound_msgq->lock();
      while ( ! __inbound_msgq->empty() ) {
	FawkesNetworkMessage *m = __inbound_msgq->front();
	wakeup_list.push_back(m->cid());
	__parent->dispatch_message(m);
	m->unref();
	__inbound_msgq->pop();
      }
      __inbound_msgq->unlock();

      lock.unlock();
    
      wakeup_list.sort();
      wakeup_list.unique();
      for (std::list<unsigned int>::iterator i = wakeup_list.begin(); i != wakeup_list.end(); ++i) {
	__parent->wake_handlers(*i);
      }
    } catch (ConnectionDiedException &e) {
      throw;
    }
  }

  virtual void once()
  {
    __parent->set_recv_slave_alive();
  }

  virtual void loop()
  {
    // just return if not connected
    if (! __s ) return;

    short p = 0;
    try {
      p = __s->poll();
    } catch (InterruptedException &e) {
      return;
    }

    if ( (p & Socket::POLL_ERR) ||
	 (p & Socket::POLL_HUP) ||
	 (p & Socket::POLL_RDHUP)) {
      __parent->connection_died();
      exit();
    } else if ( p & Socket::POLL_IN ) {
      // Data can be read
      try {
	recv();
      } catch (ConnectionDiedException &e) {
	__parent->connection_died();
	exit();
      }
    }
  }

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  StreamSocket *__s;
  FawkesNetworkClient *__parent;
  FawkesNetworkMessageQueue *  __inbound_msgq;
  Mutex *__recv_mutex;
};


/** @class FawkesNetworkClient netcomm/fawkes/client.h
 * Simple Fawkes network client. Allows access to a remote instance via the
 * network. Encapsulates all needed interaction with the network.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor.
 * @param host remote host to connect to.
 * @param port port to connect to.
 */
FawkesNetworkClient::FawkesNetworkClient(const char *host, unsigned short int port)
{
  __host = strdup(host);
  __port = port;
  addr_  = NULL;
  addr_len_ = 0;

  s = NULL;
  __send_slave = NULL;
  __recv_slave = NULL;

  connection_died_recently = false;
  __send_slave_alive = false;
  __recv_slave_alive = false;

  slave_status_mutex = new Mutex();

  _id     = 0;
  _has_id = false;

  __recv_mutex          = new Mutex();
  __recv_waitcond       = new WaitCondition(__recv_mutex);
  __connest_mutex       = new Mutex();
  __connest_waitcond    = new WaitCondition(__connest_mutex);
  __connest             = false;
  __connest_interrupted = false;
}


/** Constructor.
 * Note, you cannot call the connect() without parameters the first time you
 * establish an connection when using this ctor!
 */
FawkesNetworkClient::FawkesNetworkClient()
{
  __host = NULL;
  __port = 0;
  addr_  = NULL;
  addr_len_ = 0;

  s = NULL;
  __send_slave = NULL;
  __recv_slave = NULL;

  connection_died_recently = false;
  __send_slave_alive = false;
  __recv_slave_alive = false;

  slave_status_mutex = new Mutex();

  _id     = 0;
  _has_id = false;

  __recv_mutex          = new Mutex();
  __recv_waitcond       = new WaitCondition(__recv_mutex);
  __connest_mutex       = new Mutex();
  __connest_waitcond    = new WaitCondition(__connest_mutex);
  __connest             = false;
  __connest_interrupted = false;
}


/** Constructor.
 * @param id id of the client.
 * @param host remote host to connect to.
 * @param port port to connect to.
 */
FawkesNetworkClient::FawkesNetworkClient(unsigned int id, const char *host,
                                         unsigned short int port)
{
  __host = strdup(host);
  __port = port;
  addr_  = NULL;
  addr_len_ = 0;

  s = NULL;
  __send_slave = NULL;
  __recv_slave = NULL;

  connection_died_recently = false;
  __send_slave_alive = false;
  __recv_slave_alive = false;

  slave_status_mutex = new Mutex();

  _id     = id;
  _has_id = true;

  __recv_mutex          = new Mutex();
  __recv_waitcond       = new WaitCondition(__recv_mutex);
  __connest_mutex       = new Mutex();
  __connest_waitcond    = new WaitCondition(__connest_mutex);
  __connest             = false;
  __connest_interrupted = false;
}


/** Destructor. */
FawkesNetworkClient::~FawkesNetworkClient()
{
  disconnect();

  delete s;
  if (__host) free(__host);
  if (addr_) free(addr_);
  delete slave_status_mutex;

  delete __connest_waitcond;
  delete __connest_mutex;
  delete __recv_waitcond;
  delete __recv_mutex;
}


/** Connect to remote.
 * @exception SocketException thrown by Socket::connect()
 * @exception NullPointerException thrown if hostname has not been set
 */
void
FawkesNetworkClient::connect()
{
  if ( __host == NULL && addr_ == NULL) {
    throw NullPointerException("Neither hostname nor sockaddr set. Cannot connect.");
  }

  if ( s != NULL ) {
    disconnect();
  }


  connection_died_recently = false;

  try {
    s = new StreamSocket();
    if (addr_) {
	    s->connect(addr_, addr_len_);
    } else if (__host) {
	    s->connect(__host, __port);
    } else {
	    throw NullPointerException("Nothing to connect to!?");
    }
    __send_slave = new FawkesNetworkClientSendThread(s, this);
    __send_slave->start();
    __recv_slave = new FawkesNetworkClientRecvThread(s, this, __recv_mutex);
    __recv_slave->start();
  } catch (SocketException &e) {
    connection_died_recently = true;
    if ( __send_slave ) {
      __send_slave->cancel();
      __send_slave->join();
      delete __send_slave;
      __send_slave = NULL;
    }
    if ( __recv_slave ) {
      __recv_slave->cancel();
      __recv_slave->join();
      delete __recv_slave;
      __recv_slave = NULL;
    }
    __send_slave_alive = false;
    __recv_slave_alive = false;
    delete s;
    s = NULL;
    throw;
  }

  __connest_mutex->lock();
  while ( ! __connest && ! __connest_interrupted ) {
    __connest_waitcond->wait();
  }
  bool interrupted = __connest_interrupted;
  __connest_interrupted = false;
  __connest_mutex->unlock();
  if ( interrupted ) {
    throw InterruptedException("FawkesNetworkClient::connect()");
  }

  notify_of_connection_established();
}


/** Connect to new ip and port, and set hostname.
 * @param host remote host name
 * @param port new port to connect to
 * @see connect() Look there for more documentation and notes about possible
 * exceptions.
 */
void
FawkesNetworkClient::connect(const char *host, unsigned short int port)
{
  if (__host)  free(__host);
  __host = strdup(host);
  __port = port;
  connect();
}

/** Connect to specific endpoint.
 * @param hostname hostname, informational only and not used for connecting
 * @param addr sockaddr structure of specific endpoint to connect to
 * @param addr_len length of @p addr
 */
void
FawkesNetworkClient::connect(const char *hostname, const struct sockaddr *addr, socklen_t addr_len)
{
  if (__host)  free(__host);
  if (addr_) free(addr_);
	addr_ = (struct sockaddr *)malloc(addr_len);
	addr_len_ = addr_len;
	memcpy(addr_, addr, addr_len);
	__host = strdup(hostname);
  connect();
}

/** Connect to specific endpoint.
 * @param hostname hostname, informational only and not used for connecting
 * @param addr sockaddr_storage structure of specific endpoint to connect to
 */
void
FawkesNetworkClient::connect(const char *hostname, const struct sockaddr_storage &addr)
{
  if (__host)  free(__host);
  if (addr_) free(addr_);
  addr_ = (struct sockaddr *)malloc(sizeof(sockaddr_storage));
  addr_len_ = sizeof(sockaddr_storage);
	memcpy(addr_, &addr, addr_len_);
	__host = strdup(hostname);
  connect();
}

/** Disconnect socket. */
void
FawkesNetworkClient::disconnect()
{
  if ( s == NULL ) return;

  if ( __send_slave_alive ) {
    if ( ! connection_died_recently ) {
      __send_slave->force_send();
      // Give other side some time to read the messages just sent
      usleep(100000);
    }
    __send_slave->cancel();
    __send_slave->join();
    delete __send_slave;
    __send_slave = NULL;
  }
  if ( __recv_slave_alive ) {
    __recv_slave->cancel();
    __recv_slave->join();
    delete __recv_slave;
    __recv_slave = NULL;
  }
  __send_slave_alive = false;
  __recv_slave_alive = false;
  delete s;
  s = NULL;

  if (! connection_died_recently) {
    connection_died();
  }
}


/** Interrupt connect().
 * This is for example handy to interrupt in connection_died() before a
 * connection_established() event has been received.
 */
void
FawkesNetworkClient::interrupt_connect()
{
  __connest_mutex->lock();
  __connest_interrupted = true;
  __connest_waitcond->wake_all();
  __connest_mutex->unlock();
}


/** Enqueue message to send.
 * This method takes ownership of the message. If you want to use the message
 * after enqueing you must reference:
 * @code
 * message->ref();
 * fawkes_network_client->enqueue(message);
 * // message can now still be used
 * @endcode
 * Without extra referencing the message may not be used after enqueuing.
 * @param message message to send
 */
void
FawkesNetworkClient::enqueue(FawkesNetworkMessage *message)
{
  if (__send_slave)  __send_slave->enqueue(message);
}


/** Enqueue message to send and wait for answer. It is guaranteed that an
 * answer cannot be missed. However, if the component sends another message
 * (which is not the answer to the query) this will also trigger the wait
 * condition to be woken up. The component ID to wait for is taken from the
 * message.
 * This message also calls unref() on the message. If you want to use it
 * after enqueuing make sure you ref() before calling this method.
 * @param message message to send
 * @param timeout_sec timeout for the waiting operation in seconds, 0 to wait
 * forever (warning, this may result in a deadlock!)
 */
void
FawkesNetworkClient::enqueue_and_wait(FawkesNetworkMessage *message,
				      unsigned int timeout_sec)
{
  if (__send_slave && __recv_slave) {
    __recv_mutex->lock();
    if ( __recv_received.find(message->cid()) != __recv_received.end()) {
      __recv_mutex->unlock();
      unsigned int cid = message->cid();
      throw Exception("There is already a thread waiting for messages of "
		      "component id %u", cid);
    }
    __send_slave->enqueue(message);
    unsigned int cid = message->cid();
    __recv_received[cid] = false;
    while (!__recv_received[cid] && ! connection_died_recently) {
      if (!__recv_waitcond->reltimed_wait(timeout_sec, 0)) {
	__recv_received.erase(cid);
	__recv_mutex->unlock();
	throw TimeoutException("Timeout reached while waiting for incoming message "
			       "(outgoing was %u:%u)", message->cid(), message->msgid());
      }
    }
    __recv_received.erase(cid);
    __recv_mutex->unlock();
  } else {
    unsigned int cid = message->cid();
    unsigned int msgid = message->msgid();
    throw Exception("Cannot enqueue given message %u:%u, sender or "
		    "receiver missing", cid, msgid);
  }
}


/** Register handler.
 * Handlers are used to handle incoming packets. There may only be one handler per
 * component!
 * Cannot be called while processing a message.
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
  }
  handlers.unlock();
}


/** Deregister handler.
 * Cannot be called while processing a message.
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
  handlers.unlock();
  __recv_mutex->lock();
  if (__recv_received.find(component_id) != __recv_received.end()) {
    __recv_received[component_id] = true;
    __recv_waitcond->wake_all();
  }
  __recv_mutex->unlock();
}


void
FawkesNetworkClient::dispatch_message(FawkesNetworkMessage *m)
{
  unsigned int cid = m->cid();
  handlers.lock();
  if (handlers.find(cid) != handlers.end()) {
    handlers[cid]->inbound_received(m, _id);
  }
  handlers.unlock();
}


void
FawkesNetworkClient::wake_handlers(unsigned int cid)
{
  __recv_mutex->lock();
  if (__recv_received.find(cid) != __recv_received.end()) {
    __recv_received[cid] = true;
  }
  __recv_waitcond->wake_all();
  __recv_mutex->unlock();
}

void
FawkesNetworkClient::notify_of_connection_dead()
{
  __connest_mutex->lock();
  __connest = false;
  __connest_mutex->unlock();

  handlers.lock();
  for ( HandlerMap::iterator i = handlers.begin(); i != handlers.end(); ++i ) {
    i->second->connection_died(_id);
  }
  handlers.unlock();

  __recv_mutex->lock();
  __recv_waitcond->wake_all();
  __recv_mutex->unlock();
}

void
FawkesNetworkClient::notify_of_connection_established()
{
  handlers.lock();
  for ( HandlerMap::iterator i = handlers.begin(); i != handlers.end(); ++i ) {
    i->second->connection_established(_id);
  }
  handlers.unlock();
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
  __send_slave_alive = true;
  if ( __send_slave_alive && __recv_slave_alive ) {
    __connest_mutex->lock();
    __connest = true;
    __connest_waitcond->wake_all();
    __connest_mutex->unlock();
  }
  slave_status_mutex->unlock();
}


void
FawkesNetworkClient::set_recv_slave_alive()
{
  slave_status_mutex->lock();
  __recv_slave_alive = true;
  if ( __send_slave_alive && __recv_slave_alive ) {
    __connest_mutex->lock();
    __connest = true;
    __connest_waitcond->wake_all();
    __connest_mutex->unlock();
  }
  slave_status_mutex->unlock();
}


/** Wait for messages for component ID.
 * This will wait for messages of the given component ID to arrive. The calling
 * thread is blocked until messages are available.
 * @param component_id component ID to monitor
 * @param timeout_sec timeout for the waiting operation in seconds, 0 to wait
 * forever (warning, this may result in a deadlock!)
 */
void
FawkesNetworkClient::wait(unsigned int component_id, unsigned int timeout_sec)
{
  __recv_mutex->lock();
  if ( __recv_received.find(component_id) != __recv_received.end()) {
    __recv_mutex->unlock();
    throw Exception("There is already a thread waiting for messages of "
		    "component id %u", component_id);
  }
  __recv_received[component_id] = false;
  while (! __recv_received[component_id] && ! connection_died_recently) {
    if (!__recv_waitcond->reltimed_wait(timeout_sec, 0)) {
      __recv_received.erase(component_id);
      __recv_mutex->unlock();
      throw TimeoutException("Timeout reached while waiting for incoming message "
			     "(component %u)", component_id);
    }
  }
  __recv_received.erase(component_id);
  __recv_mutex->unlock();
}


/** Wake a waiting thread.
 * This will wakeup all threads currently waiting for the specified component ID.
 * This can be helpful to wake a sleeping thread if you received a signal.
 * @param component_id component ID for threads to wake up
 */
void
FawkesNetworkClient::wake(unsigned int component_id)
{
  __recv_mutex->lock();
  if ( __recv_received.find(component_id) != __recv_received.end()) {
    __recv_received[component_id] = true;
  }
  __recv_waitcond->wake_all();
  __recv_mutex->unlock();
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

/** Get the client's hostname
 * @return hostname or NULL
 */
const char *
FawkesNetworkClient::get_hostname() const
{
  return __host;
}

} // end namespace fawkes
