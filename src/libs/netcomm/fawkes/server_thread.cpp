
/***************************************************************************
 *  server_thread.cpp - manage Fawkes network connection
 *
 *  Created: Sun Nov 19 15:08:30 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/fawkes/server_thread.h>
#include <netcomm/fawkes/server_client_thread.h>
#include <netcomm/utils/acceptor_thread.h>
#include <netcomm/fawkes/message.h>
#include <netcomm/fawkes/handler.h>
#include <netcomm/fawkes/message_queue.h>
#include <netcomm/fawkes/message_content.h>
#include <core/threading/thread_collector.h>
#include <core/threading/mutex.h>

#include <cstdio>

/** @class FawkesNetworkServerThread <netcomm/fawkes/server_thread.h>
 * Fawkes Network Thread.
 * Maintains a list of clients and reacts on events triggered by the clients.
 * Also runs the acceptor thread.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor.
 * @param thread_collector thread collector to register new threads with
 * @param fawkes_port port for Fawkes network protocol
 */
FawkesNetworkServerThread::FawkesNetworkServerThread(ThreadCollector *thread_collector,
						     unsigned int fawkes_port)
  : Thread("FawkesNetworkServerThread", Thread::OPMODE_WAITFORWAKEUP)
{
  this->thread_collector = thread_collector;
  clients.clear();
  next_client_id = 1;
  inbound_messages = new FawkesNetworkMessageQueue();

  acceptor_thread = new NetworkAcceptorThread(this, fawkes_port,
					      "FawkesNetworkAcceptorThread");
  thread_collector->add(acceptor_thread);

}


/** Destructor. */
FawkesNetworkServerThread::~FawkesNetworkServerThread()
{
  for (cit = clients.begin(); cit != clients.end(); ++cit) {
    thread_collector->remove((*cit).second);
    delete (*cit).second;
  }
  thread_collector->remove(acceptor_thread);
  delete acceptor_thread;

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

  client->set_clid(next_client_id);
  thread_collector->add(client);
  clients.lock();
  clients[next_client_id] = client;
  clients.unlock();
  for (hit = handlers.begin(); hit != handlers.end(); ++hit) {
    (*hit).second->client_connected(next_client_id);
  }
  ++next_client_id;

  wakeup();
}


/** Add a handler.
 * @param handler to add.
 */
void
FawkesNetworkServerThread::add_handler(FawkesNetworkHandler *handler)
{
  handlers.lock();
  if ( handlers.find(handler->id()) != handlers.end()) {
    handlers.unlock();
    throw Exception("Handler already registered");
  }
  handlers[handler->id()] = handler;
  handlers.unlock();
}


/** Remove handler.
 * @param handler handler to remove
 */
void
FawkesNetworkServerThread::remove_handler(FawkesNetworkHandler *handler)
{
  handlers.lock();
  if( handlers.find(handler->id()) != handlers.end() ) {
    handlers.erase(handler->id());
  }
  handlers.unlock();
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
  clients.lock();

  // check for dead clients
  cit = clients.begin();
  while (cit != clients.end()) {
    if ( ! (*cit).second->alive() ) {
      thread_collector->remove((*cit).second);
      delete (*cit).second;
      unsigned int clid = (*cit).first;
      ++cit;
      clients.erase(clid);
      for (hit = handlers.begin(); hit != handlers.end(); ++hit) {
	(*hit).second->client_disconnected(clid);
      }
    } else {
      ++cit;
    }
  }

  // dispatch messages
  inbound_messages->lock();
  while ( ! inbound_messages->empty() ) {
    FawkesNetworkMessage *m = inbound_messages->front();
    if ( handlers.find(m->cid()) != handlers.end()) {
      handlers[m->cid()]->handle_network_message(m);
    }
    m->unref();
    inbound_messages->pop();
  }
  inbound_messages->unlock();

  clients.unlock();
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
 * Method to broadcast a message to all connected clients.
 * @param msg Message to broadcast
 */
void
FawkesNetworkServerThread::broadcast(FawkesNetworkMessage *msg)
{
  for (cit = clients.begin(); cit != clients.end(); ++cit) {
    if ( (*cit).second->alive() ) {
      (*cit).second->enqueue(msg);
    }
  }
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
FawkesNetworkServerThread::broadcast(unsigned short int component_id, unsigned short int msg_id,
			       void *payload, unsigned int payload_size)
{
  FawkesNetworkMessage *m = new FawkesNetworkMessage(component_id, msg_id,
						     payload, payload_size);
  broadcast(m);
  m->unref();
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
  m->unref();
}


/** Send a message.
 * Method to send a message to a specific client.
 * The client ID provided in the message is used to determine the correct
 * recipient. If no client is connected for the given client ID the message
 * shall be silently ignored.
 * Implemented Emitter interface message.
 * @param msg Message to send
 */
void
FawkesNetworkServerThread::send(FawkesNetworkMessage *msg)
{
  unsigned int clid = msg->clid();
  if ( clients.find(clid) != clients.end() ) {
    if ( clients[clid]->alive() ) {
      clients[clid]->enqueue(msg);
    }
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
  m->unref();
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
  m->unref();
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
  m->unref();
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


/** Call handler processing methods.
 */
void
FawkesNetworkServerThread::process()
{
  for (hit = handlers.begin(); hit != handlers.end(); ++hit) {
    (*hit).second->process_after_loop();
  }
}
