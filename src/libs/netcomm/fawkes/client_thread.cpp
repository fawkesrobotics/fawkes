
/***************************************************************************
 *  client_thread.cpp - Thread handling Fawkes network client
 *
 *  Created: Fri Nov 17 17:23:24 2006
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

#include <core/exceptions/system.h>

#include <netcomm/fawkes/client_thread.h>
#include <netcomm/fawkes/network_thread.h>
#include <netcomm/fawkes/message_queue.h>
#include <netcomm/fawkes/transceiver.h>
#include <netcomm/socket/stream.h>
#include <netcomm/utils/exceptions.h>
#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>

/** @class FawkesNetworkClientThread netcomm/fawkes/client_thread.h
 * Fawkes Network Client Thread.
 * Handles network client traffic
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param s socket to client
 * @param parent parent network thread
 */
FawkesNetworkClientThread::FawkesNetworkClientThread(StreamSocket *s,
						     FawkesNetworkThread *parent)
  : Thread("FawkesNetworkClientThread")
{
  this->s = s;
  this->parent = parent;
  _alive = true;
  _clid = 0;
  outbound_queue = new FawkesNetworkMessageQueue();
  inbound_queue = new FawkesNetworkMessageQueue();
  _outbound_mutex = new Mutex();
  _outbound_waitcond = new WaitCondition();
}


/** Destructor. */
FawkesNetworkClientThread::~FawkesNetworkClientThread()
{
  delete s;
  delete _outbound_mutex;
  delete _outbound_waitcond;
  delete outbound_queue;
  delete inbound_queue;
}


/** Get client ID.
 * The client ID can be used to send replies.
 * @return client ID
 */
unsigned int
FawkesNetworkClientThread::clid() const
{
  return _clid;
}


/** Set client ID.
 * @param client_id new client ID
 */
void
FawkesNetworkClientThread::set_clid(unsigned int client_id)
{
  _clid = client_id;
}


/** Receive data.
 * Receives data from the network if there is any and then dispatches all
 * inbound messages via the parent FawkesNetworkThread::dispatch()
 */
void
FawkesNetworkClientThread::recv()
{
  try {
    FawkesNetworkTransceiver::recv(s, inbound_queue);

    inbound_queue->lock();
    while ( ! inbound_queue->empty() ) {
      FawkesNetworkMessage *m = inbound_queue->front();
      m->set_client_id(_clid);
      parent->dispatch(m);
      m->unref();
      inbound_queue->pop();
    }
    parent->wakeup();
    inbound_queue->unlock();

  } catch (ConnectionDiedException &e) {
    _alive = false;
    s->close();
    parent->wakeup();
    exit();
  }
}


/** Thread loop.
 * The client thread loop polls on the socket for 10 ms (wait for events
 * on the socket like closed connection or data that can be read). If any
 * event occurs it is processed. If the connection died or any other
 * error occured the thread is cancelled and the parent FawkesNetworkThread
 * is woken up to carry out any action that is needed when a client dies.
 * If data is available for reading thedata is received and dispatched
 * via recv().
 * Afterwards the outbound message queue is processed and alle messages are
 * sent. This is also done if the operation could block (POLL_OUT is not
 * honored).
 */
void
FawkesNetworkClientThread::loop()
{
  
  short p = 0;
  try {
    p = s->poll(10); // block for up to 10 ms
  } catch (InterruptedException &e) {
    // we just ignore this and try it again
    return;
  }

  if ( (p & Socket::POLL_ERR) ||
       (p & Socket::POLL_HUP) ||
       (p & Socket::POLL_RDHUP)) {
    _alive = false;
    parent->wakeup();
    exit();
  } else if ( p & Socket::POLL_IN ) {
    // Data can be read
    recv();
  }

  _outbound_mutex->lock();
  if ( ! outbound_queue->empty() ) {
    try {
      FawkesNetworkTransceiver::send(s, outbound_queue);
      _outbound_mutex->unlock();
      _outbound_waitcond->wake_all();
    } catch (ConnectionDiedException &e) {
      _alive = false;
      _outbound_mutex->unlock();
      _outbound_waitcond->wake_all();
      parent->wakeup();
      exit();
    }
  } else {
    _outbound_mutex->unlock();
    _outbound_waitcond->wake_all();
  }
}

/** Enqueue message to outbound queue.
 * This enqueues the given message to the outbound queue. The message will be send
 * in the next loop iteration.
 * @param msg message to enqueue
 */
void
FawkesNetworkClientThread::enqueue(FawkesNetworkMessage *msg)
{
  msg->ref();
  outbound_queue->push_locked(msg);
}


/** Check aliveness of connection.
 * @return true if connection is still alive, false otherwise.
 */
bool
FawkesNetworkClientThread::alive() const
{
  return _alive;
}


/** Force sending of all pending outbound messages.
 * This is a blocking operation. The current poll will be interrupted by sending
 * a signal to this thread (and ignoring it) and then wait for the sending to
 * finish.
 */
void
FawkesNetworkClientThread::force_send()
{
  _outbound_mutex->lock();
  _outbound_waitcond->wait(_outbound_mutex);
  _outbound_mutex->unlock();
}
