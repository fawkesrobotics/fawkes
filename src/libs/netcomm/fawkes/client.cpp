
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

#include <core/exceptions/system.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/thread.h>
#include <core/threading/wait_condition.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/client_handler.h>
#include <netcomm/fawkes/message_queue.h>
#include <netcomm/fawkes/transceiver.h>
#include <netcomm/socket/stream.h>
#include <netcomm/utils/exceptions.h>

#include <cstdlib>
#include <cstring>
#include <list>
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
		s_                 = s;
		parent_            = parent;
		outbound_mutex_    = new Mutex();
		outbound_msgqs_[0] = new FawkesNetworkMessageQueue();
		outbound_msgqs_[1] = new FawkesNetworkMessageQueue();
		outbound_active_   = 0;
		outbound_msgq_     = outbound_msgqs_[0];
		outbound_havemore_ = false;
	}

	/** Destructor. */
	~FawkesNetworkClientSendThread()
	{
		for (unsigned int i = 0; i < 2; ++i) {
			while (!outbound_msgqs_[i]->empty()) {
				FawkesNetworkMessage *m = outbound_msgqs_[i]->front();
				m->unref();
				outbound_msgqs_[i]->pop();
			}
		}
		delete outbound_msgqs_[0];
		delete outbound_msgqs_[1];
		delete outbound_mutex_;
	}

	virtual void
	once()
	{
		parent_->set_send_slave_alive();
	}

	virtual void
	loop()
	{
		if (!parent_->connected())
			return;

		while (outbound_havemore_) {
			outbound_mutex_->lock();
			outbound_havemore_           = false;
			FawkesNetworkMessageQueue *q = outbound_msgq_;
			outbound_active_             = 1 - outbound_active_;
			outbound_msgq_               = outbound_msgqs_[outbound_active_];
			outbound_mutex_->unlock();

			if (!q->empty()) {
				try {
					FawkesNetworkTransceiver::send(s_, q);
				} catch (ConnectionDiedException &e) {
					parent_->connection_died();
					exit();
				}
			}
		}
	}

	/** Force sending of messages.
   * All messages are sent out immediately, if loop is not running already anyway.
   */
	void
	force_send()
	{
		if (loop_mutex->try_lock()) {
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
	void
	enqueue(FawkesNetworkMessage *message)
	{
		outbound_mutex_->lock();
		outbound_msgq_->push(message);
		outbound_havemore_ = true;
		outbound_mutex_->unlock();
		wakeup();
	}

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	StreamSocket *             s_;
	FawkesNetworkClient *      parent_;
	Mutex *                    outbound_mutex_;
	unsigned int               outbound_active_;
	bool                       outbound_havemore_;
	FawkesNetworkMessageQueue *outbound_msgq_;
	FawkesNetworkMessageQueue *outbound_msgqs_[2];
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
	FawkesNetworkClientRecvThread(StreamSocket *s, FawkesNetworkClient *parent, Mutex *recv_mutex)
	: Thread("FawkesNetworkClientRecvThread")
	{
		s_            = s;
		parent_       = parent;
		inbound_msgq_ = new FawkesNetworkMessageQueue();
		recv_mutex_   = recv_mutex;
	}

	/** Destructor. */
	~FawkesNetworkClientRecvThread()
	{
		while (!inbound_msgq_->empty()) {
			FawkesNetworkMessage *m = inbound_msgq_->front();
			m->unref();
			inbound_msgq_->pop();
		}
		delete inbound_msgq_;
	}

	/** Receive and process messages. */
	void
	recv()
	{
		std::list<unsigned int> wakeup_list;

		try {
			FawkesNetworkTransceiver::recv(s_, inbound_msgq_);

			MutexLocker lock(recv_mutex_);

			inbound_msgq_->lock();
			while (!inbound_msgq_->empty()) {
				FawkesNetworkMessage *m = inbound_msgq_->front();
				wakeup_list.push_back(m->cid());
				parent_->dispatch_message(m);
				m->unref();
				inbound_msgq_->pop();
			}
			inbound_msgq_->unlock();

			lock.unlock();

			wakeup_list.sort();
			wakeup_list.unique();
			for (std::list<unsigned int>::iterator i = wakeup_list.begin(); i != wakeup_list.end(); ++i) {
				parent_->wake_handlers(*i);
			}
		} catch (ConnectionDiedException &e) {
			throw;
		}
	}

	virtual void
	once()
	{
		parent_->set_recv_slave_alive();
	}

	virtual void
	loop()
	{
		// just return if not connected
		if (!s_)
			return;

		short p = 0;
		try {
			p = s_->poll();
		} catch (InterruptedException &e) {
			return;
		}

		if ((p & Socket::POLL_ERR) || (p & Socket::POLL_HUP) || (p & Socket::POLL_RDHUP)) {
			parent_->connection_died();
			exit();
		} else if (p & Socket::POLL_IN) {
			// Data can be read
			try {
				recv();
			} catch (ConnectionDiedException &e) {
				parent_->connection_died();
				exit();
			}
		}
	}

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	StreamSocket *             s_;
	FawkesNetworkClient *      parent_;
	FawkesNetworkMessageQueue *inbound_msgq_;
	Mutex *                    recv_mutex_;
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
	host_     = strdup(host);
	port_     = port;
	addr_     = NULL;
	addr_len_ = 0;

	s           = NULL;
	send_slave_ = NULL;
	recv_slave_ = NULL;

	connection_died_recently = false;
	send_slave_alive_        = false;
	recv_slave_alive_        = false;

	slave_status_mutex = new Mutex();

	_id     = 0;
	_has_id = false;

	recv_mutex_          = new Mutex();
	recv_waitcond_       = new WaitCondition(recv_mutex_);
	connest_mutex_       = new Mutex();
	connest_waitcond_    = new WaitCondition(connest_mutex_);
	connest_             = false;
	connest_interrupted_ = false;
}

/** Constructor.
 * Note, you cannot call the connect() without parameters the first time you
 * establish an connection when using this ctor!
 */
FawkesNetworkClient::FawkesNetworkClient()
{
	host_     = NULL;
	port_     = 0;
	addr_     = NULL;
	addr_len_ = 0;

	s           = NULL;
	send_slave_ = NULL;
	recv_slave_ = NULL;

	connection_died_recently = false;
	send_slave_alive_        = false;
	recv_slave_alive_        = false;

	slave_status_mutex = new Mutex();

	_id     = 0;
	_has_id = false;

	recv_mutex_          = new Mutex();
	recv_waitcond_       = new WaitCondition(recv_mutex_);
	connest_mutex_       = new Mutex();
	connest_waitcond_    = new WaitCondition(connest_mutex_);
	connest_             = false;
	connest_interrupted_ = false;
}

/** Constructor.
 * @param id id of the client.
 * @param host remote host to connect to.
 * @param port port to connect to.
 */
FawkesNetworkClient::FawkesNetworkClient(unsigned int id, const char *host, unsigned short int port)
{
	host_     = strdup(host);
	port_     = port;
	addr_     = NULL;
	addr_len_ = 0;

	s           = NULL;
	send_slave_ = NULL;
	recv_slave_ = NULL;

	connection_died_recently = false;
	send_slave_alive_        = false;
	recv_slave_alive_        = false;

	slave_status_mutex = new Mutex();

	_id     = id;
	_has_id = true;

	recv_mutex_          = new Mutex();
	recv_waitcond_       = new WaitCondition(recv_mutex_);
	connest_mutex_       = new Mutex();
	connest_waitcond_    = new WaitCondition(connest_mutex_);
	connest_             = false;
	connest_interrupted_ = false;
}

/** Destructor. */
FawkesNetworkClient::~FawkesNetworkClient()
{
	disconnect();

	delete s;
	if (host_)
		free(host_);
	if (addr_)
		free(addr_);
	delete slave_status_mutex;

	delete connest_waitcond_;
	delete connest_mutex_;
	delete recv_waitcond_;
	delete recv_mutex_;
}

/** Connect to remote.
 * @exception SocketException thrown by Socket::connect()
 * @exception NullPointerException thrown if hostname has not been set
 */
void
FawkesNetworkClient::connect()
{
	if (host_ == NULL && addr_ == NULL) {
		throw NullPointerException("Neither hostname nor sockaddr set. Cannot connect.");
	}

	if (s != NULL) {
		disconnect();
	}

	connection_died_recently = false;

	try {
		s = new StreamSocket();
		if (addr_) {
			s->connect(addr_, addr_len_);
		} else if (host_) {
			s->connect(host_, port_);
		} else {
			throw NullPointerException("Nothing to connect to!?");
		}
		send_slave_ = new FawkesNetworkClientSendThread(s, this);
		send_slave_->start();
		recv_slave_ = new FawkesNetworkClientRecvThread(s, this, recv_mutex_);
		recv_slave_->start();
	} catch (SocketException &e) {
		connection_died_recently = true;
		if (send_slave_) {
			send_slave_->cancel();
			send_slave_->join();
			delete send_slave_;
			send_slave_ = NULL;
		}
		if (recv_slave_) {
			recv_slave_->cancel();
			recv_slave_->join();
			delete recv_slave_;
			recv_slave_ = NULL;
		}
		send_slave_alive_ = false;
		recv_slave_alive_ = false;
		delete s;
		s = NULL;
		throw;
	}

	connest_mutex_->lock();
	while (!connest_ && !connest_interrupted_) {
		connest_waitcond_->wait();
	}
	bool interrupted     = connest_interrupted_;
	connest_interrupted_ = false;
	connest_mutex_->unlock();
	if (interrupted) {
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
	if (host_)
		free(host_);
	host_ = strdup(host);
	port_ = port;
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
	if (host_)
		free(host_);
	if (addr_)
		free(addr_);
	addr_     = (struct sockaddr *)malloc(addr_len);
	addr_len_ = addr_len;
	memcpy(addr_, addr, addr_len);
	host_ = strdup(hostname);
	connect();
}

/** Connect to specific endpoint.
 * @param hostname hostname, informational only and not used for connecting
 * @param addr sockaddr_storage structure of specific endpoint to connect to
 */
void
FawkesNetworkClient::connect(const char *hostname, const struct sockaddr_storage &addr)
{
	if (host_)
		free(host_);
	if (addr_)
		free(addr_);
	addr_     = (struct sockaddr *)malloc(sizeof(sockaddr_storage));
	addr_len_ = sizeof(sockaddr_storage);
	memcpy(addr_, &addr, addr_len_);
	host_ = strdup(hostname);
	connect();
}

/** Disconnect socket. */
void
FawkesNetworkClient::disconnect()
{
	if (s == NULL)
		return;

	if (send_slave_alive_) {
		if (!connection_died_recently) {
			send_slave_->force_send();
			// Give other side some time to read the messages just sent
			usleep(100000);
		}
		send_slave_->cancel();
		send_slave_->join();
		delete send_slave_;
		send_slave_ = NULL;
	}
	if (recv_slave_alive_) {
		recv_slave_->cancel();
		recv_slave_->join();
		delete recv_slave_;
		recv_slave_ = NULL;
	}
	send_slave_alive_ = false;
	recv_slave_alive_ = false;
	delete s;
	s = NULL;

	if (!connection_died_recently) {
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
	connest_mutex_->lock();
	connest_interrupted_ = true;
	connest_waitcond_->wake_all();
	connest_mutex_->unlock();
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
	if (send_slave_)
		send_slave_->enqueue(message);
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
FawkesNetworkClient::enqueue_and_wait(FawkesNetworkMessage *message, unsigned int timeout_sec)
{
	if (send_slave_ && recv_slave_) {
		recv_mutex_->lock();
		if (recv_received_.find(message->cid()) != recv_received_.end()) {
			recv_mutex_->unlock();
			unsigned int cid = message->cid();
			throw Exception("There is already a thread waiting for messages of "
			                "component id %u",
			                cid);
		}
		send_slave_->enqueue(message);
		unsigned int cid    = message->cid();
		recv_received_[cid] = false;
		while (!recv_received_[cid] && !connection_died_recently) {
			if (!recv_waitcond_->reltimed_wait(timeout_sec, 0)) {
				recv_received_.erase(cid);
				recv_mutex_->unlock();
				throw TimeoutException("Timeout reached while waiting for incoming message "
				                       "(outgoing was %u:%u)",
				                       message->cid(),
				                       message->msgid());
			}
		}
		recv_received_.erase(cid);
		recv_mutex_->unlock();
	} else {
		unsigned int cid   = message->cid();
		unsigned int msgid = message->msgid();
		throw Exception("Cannot enqueue given message %u:%u, sender or "
		                "receiver missing",
		                cid,
		                msgid);
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
                                      unsigned int                component_id)
{
	handlers.lock();
	if (handlers.find(component_id) != handlers.end()) {
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
	if (handlers.find(component_id) != handlers.end()) {
		handlers[component_id]->deregistered(_id);
		handlers.erase(component_id);
	}
	handlers.unlock();
	recv_mutex_->lock();
	if (recv_received_.find(component_id) != recv_received_.end()) {
		recv_received_[component_id] = true;
		recv_waitcond_->wake_all();
	}
	recv_mutex_->unlock();
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
	recv_mutex_->lock();
	if (recv_received_.find(cid) != recv_received_.end()) {
		recv_received_[cid] = true;
	}
	recv_waitcond_->wake_all();
	recv_mutex_->unlock();
}

void
FawkesNetworkClient::notify_of_connection_dead()
{
	connest_mutex_->lock();
	connest_ = false;
	connest_mutex_->unlock();

	handlers.lock();
	for (HandlerMap::iterator i = handlers.begin(); i != handlers.end(); ++i) {
		i->second->connection_died(_id);
	}
	handlers.unlock();

	recv_mutex_->lock();
	recv_waitcond_->wake_all();
	recv_mutex_->unlock();
}

void
FawkesNetworkClient::notify_of_connection_established()
{
	handlers.lock();
	for (HandlerMap::iterator i = handlers.begin(); i != handlers.end(); ++i) {
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
	send_slave_alive_ = true;
	if (send_slave_alive_ && recv_slave_alive_) {
		connest_mutex_->lock();
		connest_ = true;
		connest_waitcond_->wake_all();
		connest_mutex_->unlock();
	}
	slave_status_mutex->unlock();
}

void
FawkesNetworkClient::set_recv_slave_alive()
{
	slave_status_mutex->lock();
	recv_slave_alive_ = true;
	if (send_slave_alive_ && recv_slave_alive_) {
		connest_mutex_->lock();
		connest_ = true;
		connest_waitcond_->wake_all();
		connest_mutex_->unlock();
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
	recv_mutex_->lock();
	if (recv_received_.find(component_id) != recv_received_.end()) {
		recv_mutex_->unlock();
		throw Exception("There is already a thread waiting for messages of "
		                "component id %u",
		                component_id);
	}
	recv_received_[component_id] = false;
	while (!recv_received_[component_id] && !connection_died_recently) {
		if (!recv_waitcond_->reltimed_wait(timeout_sec, 0)) {
			recv_received_.erase(component_id);
			recv_mutex_->unlock();
			throw TimeoutException("Timeout reached while waiting for incoming message "
			                       "(component %u)",
			                       component_id);
		}
	}
	recv_received_.erase(component_id);
	recv_mutex_->unlock();
}

/** Wake a waiting thread.
 * This will wakeup all threads currently waiting for the specified component ID.
 * This can be helpful to wake a sleeping thread if you received a signal.
 * @param component_id component ID for threads to wake up
 */
void
FawkesNetworkClient::wake(unsigned int component_id)
{
	recv_mutex_->lock();
	if (recv_received_.find(component_id) != recv_received_.end()) {
		recv_received_[component_id] = true;
	}
	recv_waitcond_->wake_all();
	recv_mutex_->unlock();
}

/** Check if connection is alive.
 * @return true if connection is alive at the moment, false otherwise
 */
bool
FawkesNetworkClient::connected() const throw()
{
	return (!connection_died_recently && (s != NULL));
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
	if (!_has_id) {
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
	return host_;
}

} // end namespace fawkes
