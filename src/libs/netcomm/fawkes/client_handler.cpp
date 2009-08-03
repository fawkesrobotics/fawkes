
/***************************************************************************
 *  client_handler.cpp - handle incoming msgs from FawkesNetworkClientThread
 *
 *  Created: Thu Jun 14 17:18:39 2007
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

#include <netcomm/fawkes/client_handler.h>

namespace fawkes {

/** @class FawkesNetworkClientHandler <netcomm/fawkes/client_handler.h>
 * Message handler for FawkesNetworkClient.
 * This interface is used by the FawkesNetworkClient to handle incoming messages.
 * @ingroup NetComm
 * @author Tim Niemueller
 *
 * @fn virtual void FawkesNetworkClientHandler::deregistered(unsigned int id) throw() = 0
 * This handler has been deregistered.
 * This is called when this handler is deregistered from the
 * FawkesNetworkClient. Sometimes you may not want to allow this and post
 * a big fat warning into the log.
 * @param id the id of the calling client
 *
 * @fn virtual void FawkesNetworkClientHandler::inbound_received(FawkesNetworkMessage *m, unsigned int id) throw() = 0
 * Called for incoming messages.
 * This is called when an incoming message has been received. If this
 * method was called one or more times then the a previously carried out
 * wait(cid) call will continue.
 * @param m Message to handle
 * @param id the id of the calling client
 *
 * @fn virtual void FawkesNetworkClientHandler::connection_established(unsigned int id) throw() = 0
 * Client has established a connection.
 * Whenever the client establishes a connection this is signaled to handlers with this
 * method. You can register to a client at any time, you may even enqueue messages to
 * a client while the connection is dead. If the client at some point gets connected
 * again, the messages will then be send out in one go. You should use this in your
 * application though to only send data if the connection is alive and you should let
 * the user know about the connection status.
 * @param id the id of the calling client
 *
 * @fn virtual void FawkesNetworkClientHandler::connection_died(unsigned int id) throw() = 0
 * Client connection died.
 * This method is used to inform handlers that the connection has died for any reason.
 * No more data can be send and no more messages should be enqueued because it is unclear
 * when they would be sent.
 * @param id the id of the calling client
 *
 */

/** Empty virtual destructor. */
FawkesNetworkClientHandler::~FawkesNetworkClientHandler()
{
}

} // end namespace fawkes
