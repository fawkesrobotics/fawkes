
/***************************************************************************
 *  connection_dispatcher.cpp - Network connection listener and dispatcher
 *
 *  Created: Mon Oct 20 15:06:28 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <gui_utils/connection_dispatcher.h>
#include <netcomm/fawkes/client.h>

namespace fawkes {

/** @class ConnectionDispatcher <gui_utils/connection_dispatcher.h>
 * Watches network client events and dispatches them as signals.
 * @author Tim Niemueller
 */

/** Constructor. */
ConnectionDispatcher::ConnectionDispatcher()
{
  __client = new FawkesNetworkClient();
  __client->register_handler(this, FAWKES_CID_OBSERVER_MODE);
  __client_owned = true;

  __dispatcher_connected.connect(sigc::mem_fun(*this, &ConnectionDispatcher::on_connection_established));
  __dispatcher_disconnected.connect(sigc::mem_fun(*this, &ConnectionDispatcher::on_connection_died));
  __dispatcher_message_received.connect(sigc::mem_fun(*this, &ConnectionDispatcher::on_message_received));
}


/** Destructor. */
ConnectionDispatcher::~ConnectionDispatcher()
{
  set_client(NULL);
}


/** Set Fawkes network client.
 * The instance you set is not owned by the ConnectionDispatcher, it's only
 * used. You have to delete it when finished. Similarly you have to make sure that
 * the client is valid as long as it is set on the dispatcher.
 * @param client Fawkes network client to set.
 */
void
ConnectionDispatcher::set_client(FawkesNetworkClient *client)
{
  if ( __client )  __client->deregister_handler(FAWKES_CID_OBSERVER_MODE);
  if ( __client_owned ) {
    delete __client;
  }
  __client_owned = false;
  __client = client;
  if ( __client )  __client->register_handler(this, FAWKES_CID_OBSERVER_MODE);
}


/** Get client.
 * @return associated Fawkes network client.
 */
FawkesNetworkClient *
ConnectionDispatcher::get_client()
{
  return __client;
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
ConnectionDispatcher::on_connection_established()
{
  __signal_connected.emit();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
ConnectionDispatcher::on_connection_died()
{
  __signal_disconnected.emit();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
ConnectionDispatcher::on_message_received()
{
  __queue_message_received.lock();
  while (! __queue_message_received.empty()) {
    FawkesNetworkMessage *msg = __queue_message_received.front();
    __signal_message_received.emit(msg);
    msg->unref();
    __queue_message_received.pop();
  }
  __queue_message_received.unlock();
}


void
ConnectionDispatcher::deregistered(unsigned int id) throw()
{
  // ignored
}


void
ConnectionDispatcher::inbound_received(FawkesNetworkMessage *m, unsigned int id) throw()
{
  m->ref();
  __queue_message_received.push_locked(m);
  __dispatcher_message_received();
}


void
ConnectionDispatcher::connection_died(unsigned int id) throw()
{
  __dispatcher_disconnected();
}


void
ConnectionDispatcher::connection_established(unsigned int id) throw()
{
  __dispatcher_connected();
}


/** Get "message received" signal.
 * The "message received" signal is emitted whenever a FawkesNetworkMessage has
 * been received.
 * @return "message received" signal
 */
  sigc::signal<void, FawkesNetworkMessage *>
ConnectionDispatcher::signal_message_received()
{
  return __signal_message_received;
}


/** Get "connected" signal.
 * The "connected" signal is emitted when the connection has been established.
 * @return "connected" signal
 */
sigc::signal<void>
ConnectionDispatcher::signal_connected()
{
  return __signal_connected;
}


/** Get "disconnected" signal.
 * The "disconnected" signal is emitted when the connection has died, for example
 * because the other peer closed the connection.
 * @return "disconnected" signal
 */
sigc::signal<void>
ConnectionDispatcher::signal_disconnected()
{
  return __signal_disconnected;
}

} // end of namespace fawkes
