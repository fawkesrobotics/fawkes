
/***************************************************************************
 *  network_thread.cpp - manage Fawkes network connection
 *
 *  Generated: Sun Nov 19 15:08:30 2006
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

#include <netcomm/fawkes/network_thread.h>
#include <netcomm/fawkes/client_thread.h>
#include <netcomm/fawkes/acceptor_thread.h>
#include <netcomm/fawkes/message.h>
#include <netcomm/fawkes/handler.h>
#include <netcomm/fawkes/message_queue.h>
#include <core/threading/thread_collector.h>
#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>

#include <stdio.h>

/** @class FawkesNetworkThread netcomm/fawkes/network_thread.h
 * Fawkes Network Thread.
 * Maintains a list of clients and reacts on events triggered by the clients.
 * Also runs the acceptor thread.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param thread_collector thread collector to register new threads with
 * @param fawkes_port port for Fawkes network protocol
 */
FawkesNetworkThread::FawkesNetworkThread(ThreadCollector *thread_collector,
					 unsigned int fawkes_port)
  : Thread("FawkesNetworkThread")
{
  this->thread_collector = thread_collector;
  clients.clear();
  next_client_id = 1;
  clients_mutex = new Mutex();
  handlers_mutex = new Mutex();
  wait_mutex = new Mutex();
  wait_cond = new WaitCondition();
  inbound_messages = new FawkesNetworkMessageQueue();

  acceptor_thread = new FawkesNetworkAcceptorThread(this, fawkes_port);
  thread_collector->add(acceptor_thread);

}


/** Destructor. */
FawkesNetworkThread::~FawkesNetworkThread()
{
  for (cit = clients.begin(); cit != clients.end(); ++cit) {
    thread_collector->remove((*cit).second);
    delete (*cit).second;
  }
  thread_collector->remove(acceptor_thread);
  delete acceptor_thread;

  delete inbound_messages;

  delete clients_mutex;
  delete handlers_mutex;
  delete wait_mutex;
  delete wait_cond;
}


/** Add a new client.
 * Called by the FawkesNetworkAcceptorThread if a new client connected.
 * @param client new client
 */
void
FawkesNetworkThread::add_client(FawkesNetworkClientThread *client)
{
  clients_mutex->lock();
  client->setClientID(next_client_id);
  thread_collector->add(client);
  clients[next_client_id] = client;
  for (hit = handlers.begin(); hit != handlers.end(); ++hit) {
    (*hit).second->clientConnected(next_client_id);
  }
  ++next_client_id;
  clients_mutex->unlock();
}


/** Add a handler.
 * @param handler to add.
 */
void
FawkesNetworkThread::add_handler(FawkesNetworkHandler *handler)
{
  handlers_mutex->lock();
  if ( handlers.find(handler->id()) != handlers.end()) {
    handlers_mutex->unlock();
    throw Exception("Handler already registered");
  }
  handlers[handler->id()] = handler;
  handlers_mutex->unlock();
}


/** Remove handler.
 * @param handler handler to remove
 */
void
FawkesNetworkThread::remove_handler(FawkesNetworkHandler *handler)
{
  handlers_mutex->lock();
  if( handlers.find(handler->id()) != handlers.end() ) {
    handlers.erase(handler->id());
  }
  handlers_mutex->unlock();
}


/** Fawkes network thread loop.
 * The thread loop will check all clients for their alivness and dead
 * clients are removed. Then inbound messages are processed and dispatched
 * properly to registered handlers. Then the thread waits for a new event
 * to happen (event emitting threads need to wakeup this thread!).
 */
void
FawkesNetworkThread::loop()
{
  clients_mutex->lock();

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
	(*hit).second->clientDisconnected(clid);
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
      handlers[m->cid()]->handleNetworkMessage(m);
    }
    m->unref();
    inbound_messages->pop();
  }
  inbound_messages->unlock();

  clients_mutex->unlock();
  wait_mutex->lock();
  wait_cond->wait(wait_mutex);
  wait_mutex->unlock();
}


/** Wakeup this thread. */
void
FawkesNetworkThread::wakeup()
{
  wait_cond->wakeAll();
}


/** Broadcast a message.
 * Implemented Emitter interface message.
 * @param msg Message to broadcast
 * @see FawkesNetworkEmitter::broadcast()
 */
void
FawkesNetworkThread::broadcast(FawkesNetworkMessage *msg)
{
  for (cit = clients.begin(); cit != clients.end(); ++cit) {
    if ( (*cit).second->alive() ) {
      (*cit).second->enqueue(msg);
    }
  }
}


/** Send a message.
 * Implemented Emitter interface message.
 * @param msg Message to send
 * @see FawkesNetworkEmitter::send()
 */
void
FawkesNetworkThread::send(FawkesNetworkMessage *msg)
{
  unsigned int clid = msg->clid();
  if ( clients.find(clid) != clients.end() ) {
    if ( clients[clid]->alive() ) {
      clients[clid]->enqueue(msg);
    }
  }
}


/** Dispatch messages.
 * Actually messages are just put into the inbound message queue and dispatched
 * during the next loop iteration. So after adding all the messages you have
 * to wakeup the thread to get them actually dispatched.
 * @param msg message to dispatch
 */
void
FawkesNetworkThread::dispatch(FawkesNetworkMessage *msg)
{
  msg->ref();
  inbound_messages->push_locked(msg);
}


/** Call handler processing methods.
 */
void
FawkesNetworkThread::process()
{
  for (hit = handlers.begin(); hit != handlers.end(); ++hit) {
    (*hit).second->processAfterLoop();
  }
}
