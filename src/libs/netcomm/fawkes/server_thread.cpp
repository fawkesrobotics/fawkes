
/***************************************************************************
 *  server_thread.cpp - Fawkes Network Protocol (server part)
 *
 *  Created: Sun Nov 19 15:08:30 2006
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

#include <netcomm/fawkes/server_thread.h>
#include <netcomm/fawkes/server_client_thread.h>
#include <netcomm/utils/acceptor_thread.h>
#include <netcomm/fawkes/message.h>
#include <netcomm/fawkes/handler.h>
#include <netcomm/fawkes/message_queue.h>
#include <netcomm/fawkes/message_content.h>
#include <core/threading/thread_collector.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/exception.h>

#include <unistd.h>

namespace fawkes {

/** @class FawkesNetworkServerThread <netcomm/fawkes/server_thread.h>
 * Fawkes Network Thread.
 * Maintains a list of clients and reacts on events triggered by the clients.
 * Also runs the acceptor thread.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor.
 * @param enable_ipv4 true to listen on the IPv4 TCP port
 * @param enable_ipv6 true to listen on the IPv6 TCP port
 * @param listen_ipv4 IPv4 address to listen on for incoming connections,
 * 0.0.0.0 to listen on any local address
 * @param listen_ipv6 IPv6 address to listen on for incoming connections,
 * :: to listen on any local address
 * @param fawkes_port port for Fawkes network protocol
 * @param thread_collector thread collector to register new threads with
 */
FawkesNetworkServerThread::FawkesNetworkServerThread(bool enable_ipv4, bool enable_ipv6,
                                                     const std::string &listen_ipv4, const std::string &listen_ipv6,
                                                     unsigned int fawkes_port,
                                                     ThreadCollector *thread_collector)
  : Thread("FawkesNetworkServerThread", Thread::OPMODE_WAITFORWAKEUP)
{
  this->thread_collector = thread_collector;
  clients.clear();
  next_client_id = 1;
  inbound_messages = new FawkesNetworkMessageQueue();

  if (enable_ipv4) {
	  acceptor_threads.push_back(new NetworkAcceptorThread(this, Socket::IPv4, listen_ipv4, fawkes_port,
	                                                       "FawkesNetworkAcceptorThread"));
  }
  if (enable_ipv6) {
	  acceptor_threads.push_back(new NetworkAcceptorThread(this, Socket::IPv6, listen_ipv6, fawkes_port,
	                                                       "FawkesNetworkAcceptorThread"));
  }
		  
  if ( thread_collector ) {
	  for (size_t i = 0; i < acceptor_threads.size(); ++i) {
		  thread_collector->add(acceptor_threads[i]);
	  }
  } else {
	  for (size_t i = 0; i < acceptor_threads.size(); ++i) {
		  acceptor_threads[i]->start();
	  }
  }
}


/** Destructor. */
FawkesNetworkServerThread::~FawkesNetworkServerThread()
{
  for (cit = clients.begin(); cit != clients.end(); ++cit) {
    if ( thread_collector ) {
      thread_collector->remove((*cit).second);
    } else {
      (*cit).second->cancel();
      (*cit).second->join();
    }
    delete (*cit).second;
  }
  for (size_t i = 0; i < acceptor_threads.size(); ++i) {
	  if ( thread_collector ) {
		  thread_collector->remove(acceptor_threads[i]);
	  } else {
		  acceptor_threads[i]->cancel();
		  acceptor_threads[i]->join();
	  }
	  delete acceptor_threads[i];
  }
  acceptor_threads.clear();

  delete inbound_messages;
}


/** Add a new connection.
 * Called by the NetworkAcceptorThread if a new client connected.
 * @param s socket for new client
 */
void
FawkesNetworkServerThread::add_connection(StreamSocket *s) throw()
{
  FawkesNetworkServerClientThread *client = new FawkesNetworkServerClientThread(s, this);

  clients.lock();
  client->set_clid(next_client_id);
  if ( thread_collector ) {
    thread_collector->add(client);
  } else {
    client->start();
  }
  unsigned int cid = next_client_id++;
  clients[cid] = client;
  clients.unlock();

  MutexLocker handlers_lock(handlers.mutex());
  for (hit = handlers.begin(); hit != handlers.end(); ++hit) {
    (*hit).second->client_connected(cid);
  }
  handlers_lock.unlock();

  wakeup();
}


/** Add a handler.
 * @param handler to add.
 */
void
FawkesNetworkServerThread::add_handler(FawkesNetworkHandler *handler)
{
  MutexLocker handlers_lock(handlers.mutex());
  if ( handlers.find(handler->id()) != handlers.end()) {
    throw Exception("Handler already registered");
  }
  handlers[handler->id()] = handler;
}


/** Remove handler.
 * @param handler handler to remove
 */
void
FawkesNetworkServerThread::remove_handler(FawkesNetworkHandler *handler)
{
  MutexLocker handlers_lock(handlers.mutex());
  if( handlers.find(handler->id()) != handlers.end() ) {
    handlers.erase(handler->id());
  }
}


/** Fawkes network thread loop.
 * The thread loop will check all clients for their alivness and dead
 * clients are removed. Then inbound messages are processed and dispatched
 * properly to registered handlers. Then the thread waits for a new event
 * to happen (event emitting threads need to wakeup this thread!).
 */
void
FawkesNetworkServerThread::loop()
{
  std::list<unsigned int> dead_clients;
  clients.lock();
  // check for dead clients
  for (cit = clients.begin(); cit != clients.end(); ++cit) {
    if ( ! cit->second->alive() ) {
	    dead_clients.push_back(cit->first);
    }
  }
  clients.unlock();

  std::list<unsigned int>::iterator dci;
  for (dci = dead_clients.begin(); dci != dead_clients.end(); ++dci) {
	  const unsigned int clid = *dci;
	  
	  {
		  MutexLocker handlers_lock(handlers.mutex());
		  for (hit = handlers.begin(); hit != handlers.end(); ++hit) {
			  (*hit).second->client_disconnected(clid);
		  }
	  }

	  {
		  MutexLocker clients_lock(clients.mutex());
		  if ( thread_collector ) {
			  thread_collector->remove(clients[clid]);
		  } else {
			  clients[clid]->cancel();
			  clients[clid]->join();
		  }
		  usleep(5000);
		  delete clients[clid];
      clients.erase(clid);
	  }
  }

  // dispatch messages
  inbound_messages->lock();
  while ( ! inbound_messages->empty() ) {
    FawkesNetworkMessage *m = inbound_messages->front();
    {
	    MutexLocker handlers_lock(handlers.mutex());
	    if ( handlers.find(m->cid()) != handlers.end()) {
		    handlers[m->cid()]->handle_network_message(m);
	    }
    }
    m->unref();
    inbound_messages->pop();
  }
  inbound_messages->unlock();
}


/** Force sending of all pending messages. */
void
FawkesNetworkServerThread::force_send()
{
  clients.lock();
  for (cit = clients.begin(); cit != clients.end(); ++cit) {
    (*cit).second->force_send();
  }
  clients.unlock();
}


/** Broadcast a message.
 * Method to broadcast a message to all connected clients. This method will take
 * ownership of the passed message. If you want to use if after enqueing it you
 * must reference it explicitly before calling this method.
 * @param msg Message to broadcast
 */
void
FawkesNetworkServerThread::broadcast(FawkesNetworkMessage *msg)
{
  clients.lock();
  for (cit = clients.begin(); cit != clients.end(); ++cit) {
    if ( (*cit).second->alive() ) {
      msg->ref();
      (*cit).second->enqueue(msg);
    }
  }
  clients.unlock();
  msg->unref();
}


/** Broadcast a message.
 * A FawkesNetworkMessage is created and broacasted via the emitter.
 * @param component_id component ID
 * @param msg_id message type id
 * @param payload payload buffer
 * @param payload_size size of payload buffer
 * @see FawkesNetworkEmitter::broadcast()
 */
void
FawkesNetworkServerThread::broadcast(unsigned short int component_id,
				     unsigned short int msg_id,
				     void *payload, unsigned int payload_size)
{
  FawkesNetworkMessage *m = new FawkesNetworkMessage(component_id, msg_id,
						     payload, payload_size);
  broadcast(m);
}


/** Broadcast message without payload.
 * @param component_id component ID
 * @param msg_id message type ID
 */
void
FawkesNetworkServerThread::broadcast(unsigned short int component_id, unsigned short int msg_id)
{
  FawkesNetworkMessage *m = new FawkesNetworkMessage(component_id, msg_id);
  broadcast(m);
}


/** Send a message.
 * Method to send a message to a specific client.
 * The client ID provided in the message is used to determine the correct
 * recipient. If no client is connected for the given client ID the message
 * shall be silently ignored.
 * This method will take ownership of the passed message. If you want to use
 * if after enqueing it you must reference it explicitly before calling this
 * method.
 * Implemented Emitter interface message.
 * @param msg Message to send
 */
void
FawkesNetworkServerThread::send(FawkesNetworkMessage *msg)
{
	MutexLocker lock(clients.mutex());
	unsigned int clid = msg->clid();
  if ( clients.find(clid) != clients.end() ) {
    if ( clients[clid]->alive() ) {
      clients[clid]->enqueue(msg);
    } else {
      throw Exception("Client %u not alive", clid);
    }
  } else {
    throw Exception("Client %u not found", clid);
  }
}


/** Send a message.
 * A FawkesNetworkMessage is created and sent via the emitter.
 * @param to_clid client ID of recipient
 * @param component_id component ID
 * @param msg_id message type id
 * @param payload payload buffer
 * @param payload_size size of payload buffer
 * @see FawkesNetworkEmitter::broadcast()
 */
void
FawkesNetworkServerThread::send(unsigned int to_clid,
			  unsigned short int component_id, unsigned short int msg_id,
			   void *payload, unsigned int payload_size)
{
  FawkesNetworkMessage *m = new FawkesNetworkMessage(to_clid, component_id, msg_id,
						     payload, payload_size);
  send(m);
}


/** Send a message.
 * A FawkesNetworkMessage is created and sent via the emitter.
 * @param to_clid client ID of recipient
 * @param component_id component ID
 * @param msg_id message type id
 * @param content Fawkes complex network message content
 * @see FawkesNetworkEmitter::broadcast()
 */
void
FawkesNetworkServerThread::send(unsigned int to_clid,
			  unsigned short int component_id, unsigned short int msg_id,
			  FawkesNetworkMessageContent *content)
{
  FawkesNetworkMessage *m = new FawkesNetworkMessage(to_clid, component_id, msg_id,
						     content);
  send(m);
}


/** Send a message without payload.
 * A FawkesNetworkMessage with empty payload is created and sent via the emitter.
 * This is particularly useful for simple status messages that you want to send.
 * @param to_clid client ID of recipient
 * @param component_id component ID
 * @param msg_id message type id
 * @see FawkesNetworkEmitter::broadcast()
 */
void
FawkesNetworkServerThread::send(unsigned int to_clid,
			  unsigned short int component_id, unsigned short int msg_id)
{
  FawkesNetworkMessage *m = new FawkesNetworkMessage(to_clid, component_id, msg_id);
  send(m);
}


/** Dispatch messages.
 * Actually messages are just put into the inbound message queue and dispatched
 * during the next loop iteration. So after adding all the messages you have
 * to wakeup the thread to get them actually dispatched.
 * @param msg message to dispatch
 */
void
FawkesNetworkServerThread::dispatch(FawkesNetworkMessage *msg)
{
  msg->ref();
  inbound_messages->push_locked(msg);
}

} // end namespace fawkes
