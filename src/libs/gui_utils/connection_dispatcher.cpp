
/***************************************************************************
 *  connection_dispatcher.cpp - Network connection listener and dispatcher
 *
 *  Created: Mon Oct 20 15:06:28 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class ConnectionDispatcher <gui_utils/connection_dispatcher.h>
 * Watches network client events and dispatches them as signals.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cid component ID to register this dispatcher for. This is relevant if
 * you want to use the message received signal!
 */
ConnectionDispatcher::ConnectionDispatcher(unsigned int cid)
{
  cid_ = cid;
  client_ = new FawkesNetworkClient();
  client_->register_handler(this, cid_);
  client_owned_ = true;

  connect_signals();
}


/** Constructor.
 * @param cid component ID to register this dispatcher for. This is relevant if
 * you want to use the message received signal!
 * @param hostname hostname to connect to
 * @param port port to connect to
 */
  ConnectionDispatcher::ConnectionDispatcher(const char *hostname,
					     unsigned short int port,
					     unsigned int cid)
{
  cid_ = cid;
  client_ = new FawkesNetworkClient(hostname, port);
  client_->register_handler(this, cid_);
  client_owned_ = true;

  connect_signals();
}

/** Destructor. */
ConnectionDispatcher::~ConnectionDispatcher()
{
  set_client(NULL);
}


void
ConnectionDispatcher::connect_signals()
{
  dispatcher_connected_.connect(sigc::mem_fun(*this, &ConnectionDispatcher::on_connection_established));
  dispatcher_disconnected_.connect(sigc::mem_fun(*this, &ConnectionDispatcher::on_connection_died));
  dispatcher_message_received_.connect(sigc::mem_fun(*this, &ConnectionDispatcher::on_message_received));
}

/** Set component ID.
 * Set the component ID you want to register this connection dispatcher on. By
 * default the connection dispatcher uses the observer mode to only provide
 * connection status signals. If you want to use the dispatcher to be signaled
 * for incoming messages you have to set the appropriate component ID.
 * @param cid component ID
 */
void
ConnectionDispatcher::set_cid(unsigned int cid)
{
  if ( client_ ) {
    client_->deregister_handler(cid_);
    client_->register_handler(this, cid);
  }
  cid_ = cid;
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
  if ( client_ )  client_->deregister_handler(cid_);
  if ( client_owned_ ) {
    delete client_;
  }
  client_owned_ = false;
  client_ = client;
  if ( client_ )  client_->register_handler(this, cid_);
}


/** Get client.
 * @return associated Fawkes network client.
 */
FawkesNetworkClient *
ConnectionDispatcher::get_client()
{
  return client_;
}


/** Check if client is set and connection has been established.
 * @return true if a client exists and a connection is established, false
 * otherwise.
 */
ConnectionDispatcher::operator bool()
{
  return (client_ && client_->connected());
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
ConnectionDispatcher::on_connection_established()
{
  signal_connected_.emit();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
ConnectionDispatcher::on_connection_died()
{
  signal_disconnected_.emit();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
ConnectionDispatcher::on_message_received()
{
  queue_message_received_.lock();
  while (! queue_message_received_.empty()) {
    FawkesNetworkMessage *msg = queue_message_received_.front();
    signal_message_received_.emit(msg);
    msg->unref();
    queue_message_received_.pop();
  }
  queue_message_received_.unlock();
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
  queue_message_received_.push_locked(m);
  dispatcher_message_received_();
}


void
ConnectionDispatcher::connection_died(unsigned int id) throw()
{
  dispatcher_disconnected_();
}


void
ConnectionDispatcher::connection_established(unsigned int id) throw()
{
  dispatcher_connected_();
}


/** Get "message received" signal.
 * The "message received" signal is emitted whenever a FawkesNetworkMessage has
 * been received.
 * @return "message received" signal
 */
  sigc::signal<void, FawkesNetworkMessage *>
ConnectionDispatcher::signal_message_received()
{
  return signal_message_received_;
}


/** Get "connected" signal.
 * The "connected" signal is emitted when the connection has been established.
 * @return "connected" signal
 */
sigc::signal<void>
ConnectionDispatcher::signal_connected()
{
  return signal_connected_;
}


/** Get "disconnected" signal.
 * The "disconnected" signal is emitted when the connection has died, for example
 * because the other peer closed the connection.
 * @return "disconnected" signal
 */
sigc::signal<void>
ConnectionDispatcher::signal_disconnected()
{
  return signal_disconnected_;
}

} // end of namespace fawkes
