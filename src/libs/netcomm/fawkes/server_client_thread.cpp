
/***************************************************************************
 *  server_client_thread.cpp - Thread handling Fawkes network client
 *
 *  Created: Fri Nov 17 17:23:24 2006
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/fawkes/server_client_thread.h>
#include <netcomm/fawkes/server_thread.h>
#include <netcomm/fawkes/message_queue.h>
#include <netcomm/fawkes/transceiver.h>
#include <netcomm/socket/stream.h>
#include <netcomm/utils/exceptions.h>
#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>

#include <unistd.h>

namespace fawkes {

/** @class FawkesNetworkServerClientSendThread <netcomm/fawkes/server_client_thread.h>
 * Sending thread for a Fawkes client connected to the server.
 * This thread is spawned for each client connected to the server to handle the
 * server-side sending
 * @ingroup NetComm
 * @author Tim Niemueller
 */

class FawkesNetworkServerClientSendThread
  : public Thread
{
 public:
  /** Constructor.
   * @param s client stream socket
   * @param parent parent FawkesNetworkServerClientThread instance
   */
  FawkesNetworkServerClientSendThread(StreamSocket *s,
				      FawkesNetworkServerClientThread *parent)
    : Thread("FawkesNetworkServerClientSendThread", Thread::OPMODE_WAITFORWAKEUP)
  {
    __s = s;
    __parent = parent;
    __outbound_mutex    = new Mutex();
    __outbound_msgqs[0] = new FawkesNetworkMessageQueue();
    __outbound_msgqs[1] = new FawkesNetworkMessageQueue();
    __outbound_active   = 0;
    __outbound_msgq     = __outbound_msgqs[0];
  }

  /** Destructor. */
  ~FawkesNetworkServerClientSendThread()
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

  virtual void loop()
  {
    if ( ! __parent->alive() )  return;

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


  /** Enqueue message to outbound queue.
   * This enqueues the given message to the outbound queue. The message will
   * be sent in the next loop iteration. This method takes ownership of the
   * transmitted message. If you want to use the message after enqueuing you
   * must reference it explicitly.
   * @param msg message to enqueue
   */
  void enqueue(FawkesNetworkMessage *msg)
  {
    __outbound_mutex->lock();
    __outbound_msgq->push(msg);
    __outbound_havemore = true;
    __outbound_mutex->unlock();
    wakeup();
  }


  /** Wait until all data has been sent. */
  void wait_for_all_sent()
  {
    loop_mutex->lock();
    loop_mutex->unlock();
  }

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  StreamSocket                    *__s;
  FawkesNetworkServerClientThread *__parent;

  Mutex                     *__outbound_mutex;
  unsigned int               __outbound_active;
  bool                       __outbound_havemore;
  FawkesNetworkMessageQueue *__outbound_msgq;
  FawkesNetworkMessageQueue *__outbound_msgqs[2];

};


/** @class FawkesNetworkServerClientThread netcomm/fawkes/server_client_thread.h
 * Fawkes Network Client Thread for server.
 * The FawkesNetworkServerThread spawns an instance of this class for every incoming
 * connection. It is then used to handle the client.
 * The thread will start another thread, an instance of
 * FawkesNetworkServerClientSendThread. This will be used to handle all outgoing
 * traffic.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor.
 * @param s socket to client
 * @param parent parent network thread
 */
FawkesNetworkServerClientThread::FawkesNetworkServerClientThread(StreamSocket *s,
								 FawkesNetworkServerThread *parent)
  : Thread("FawkesNetworkServerClientThread")
{
  _s = s;
  _parent = parent;
  _alive = true;
  _clid = 0;
  _inbound_queue = new FawkesNetworkMessageQueue();

  _send_slave = new FawkesNetworkServerClientSendThread(_s, this);

  set_prepfin_conc_loop(true);
}


/** Destructor. */
FawkesNetworkServerClientThread::~FawkesNetworkServerClientThread()
{
  _send_slave->cancel();
  _send_slave->join();
  delete _send_slave;
  delete _s;
  delete _inbound_queue;
}


/** Get client ID.
 * The client ID can be used to send replies.
 * @return client ID
 */
unsigned int
FawkesNetworkServerClientThread::clid() const
{
  return _clid;
}


/** Set client ID.
 * @param client_id new client ID
 */
void
FawkesNetworkServerClientThread::set_clid(unsigned int client_id)
{
  _clid = client_id;
}


/** Receive data.
 * Receives data from the network if there is any and then dispatches all
 * inbound messages via the parent FawkesNetworkThread::dispatch()
 */
void
FawkesNetworkServerClientThread::recv()
{
  try {
    FawkesNetworkTransceiver::recv(_s, _inbound_queue);

    _inbound_queue->lock();
    while ( ! _inbound_queue->empty() ) {
      FawkesNetworkMessage *m = _inbound_queue->front();
      m->set_client_id(_clid);
      _parent->dispatch(m);
      m->unref();
      _inbound_queue->pop();
    }
    _parent->wakeup();
    _inbound_queue->unlock();

  } catch (ConnectionDiedException &e) {
    _alive = false;
    _s->close();
    _parent->wakeup();
  }
}


void
FawkesNetworkServerClientThread::once()
{
  _send_slave->start();
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
FawkesNetworkServerClientThread::loop()
{
  if ( ! _alive) {
    usleep(1000000);
    return;
  }

  short p = 0;
  try {
    p = _s->poll(); // block until we got a message
  } catch (InterruptedException &e) {
    // we just ignore this and try it again
    return;
  }

  if ( (p & Socket::POLL_ERR) ||
       (p & Socket::POLL_HUP) ||
       (p & Socket::POLL_RDHUP)) {
    _alive = false;
    _parent->wakeup();
  } else if ( p & Socket::POLL_IN ) {
    // Data can be read
    recv();
  }
}

/** Enqueue message to outbound queue.
 * This enqueues the given message to the outbound queue. The message will be send
 * in the next loop iteration.
 * @param msg message to enqueue
 */
void
FawkesNetworkServerClientThread::enqueue(FawkesNetworkMessage *msg)
{
  _send_slave->enqueue(msg);
}


/** Check aliveness of connection.
 * @return true if connection is still alive, false otherwise.
 */
bool
FawkesNetworkServerClientThread::alive() const
{
  return _alive;
}


/** Force sending of all pending outbound messages.
 * This is a blocking operation. The current poll will be interrupted by sending
 * a signal to this thread (and ignoring it) and then wait for the sending to
 * finish.
 */
void
FawkesNetworkServerClientThread::force_send()
{
  _send_slave->wait_for_all_sent();
}


/** Connection died notification.
 * To be called only be the send slave thread.
 */
void
FawkesNetworkServerClientThread::connection_died()
{
  _alive = false;
  _parent->wakeup();
}

} // end namespace fawkes
