
/***************************************************************************
 *  hub.cpp - Fawkes network hub
 *
 *  Created: Mon May 07 19:16:34 2007
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

#include <netcomm/fawkes/hub.h>

namespace fawkes {

/** @class FawkesNetworkHub netcomm/fawkes/hub.h
 * Fawkes Network Hub.
 * This interface is the main entry point for applications, plugins and
 * threads that use the Fawkes network protocol. The hub provides means for
 * broadcasting messages to all connected clients, for sending messages
 * to a specific connected client and to add and remove handlers to process
 * incoming messages.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 *
 * @fn void FawkesNetworkHub::broadcast(FawkesNetworkMessage *msg) = 0
 * Method to broadcast a message to all connected clients.
 * This method shall be implemented thus that the message is sent to all
 * connected clients.
 * @param msg message to send.
 *
 * @fn void FawkesNetworkHub::broadcast(unsigned short int component_id, unsigned short int msg_id, void *payload, unsigned int payload_size) = 0
 * This is an overloaded member function, provided for convenience. It
 * differs from the above function only in what arguments it accepts.
 * A FawkesNetworkMessage will be created transparently and broadcasted.
 * @param component_id component id
 * @param msg_id message id
 * @param payload buffer with payload
 * @param payload_size payload size
 *
 * @fn void FawkesNetworkHub::broadcast(unsigned short int component_id, unsigned short int msg_id) = 0
 * This is an overloaded member function, provided for convenience. It
 * differs from the above function only in what arguments it accepts.
 * A FawkesNetworkMessage will be created transparently and broadcasted.
 * This can be used for messages without payload.
 * @param component_id component id
 * @param msg_id message id
 *
 * @fn void FawkesNetworkHub::send(FawkesNetworkMessage *msg) = 0
 * Method to send a message to a specific client. The recipient has to
 * be specified in the message or sending the message will fail.
 * @param msg message to send
 *
 * @fn void FawkesNetworkHub::send(unsigned int to_clid, unsigned short int component_id, unsigned short int msg_id) = 0
 * This is an overloaded member function, provided for convenience. It
 * differs from the above function only in what arguments it accepts.
 * A FawkesNetworkMessage will be created transparently and send to
 * the client with the given ID.
 * This can be used for messages without payload.
 * @param to_clid client ID of recipient
 * @param component_id component id
 * @param msg_id message ID
 *
 * @fn void FawkesNetworkHub::send(unsigned int to_clid, unsigned short int component_id, unsigned short int msg_id, void *payload, unsigned int payload_size) = 0
 * This is an overloaded member function, provided for convenience. It
 * differs from the above function only in what arguments it accepts.
 * A FawkesNetworkMessage will be created transparently and send to
 * the client with the given ID.
 * @param to_clid client ID of recipient
 * @param component_id component id
 * @param msg_id message id
 * @param payload buffer with payload
 * @param payload_size payload size
 *
 * @fn void FawkesNetworkHub::send(unsigned int to_clid, unsigned short int component_id, unsigned short int msg_id, FawkesNetworkMessageContent *content) = 0
 * This is an overloaded member function, provided for convenience. It
 * differs from the above function only in what arguments it accepts.
 * A FawkesNetworkMessage will be created transparently and send to
 * the client with the given ID.
 * @param to_clid client ID of recipient
 * @param component_id component id
 * @param msg_id message id
 * @param content complex message content
 *
 * @fn void FawkesNetworkHub::add_handler(FawkesNetworkHandler *handler) = 0
 * Add a message handler.
 * This message handler is called for incoming messages that have an appropriate
 * component ID (which is supplied by the handler).
 * @param handler handler to add
 *
 * @fn void FawkesNetworkHub::remove_handler(FawkesNetworkHandler *handler) = 0
 * Remove a message handler.
 * The message handler is removed from the list of handlers and is no longer
 * called for incoming data.
 * @param handler handler to remove
 *
 * @fn void FawkesNetworkHub::force_send() = 0
 * Force sending of all pending messages.
 * This will order the sending of all pending outbound messages that are currently
 * enqueued for clients. The method will block until this is done.
 * It is not ensured that no messages are added during that time. Make sure that
 * the call constraints guarantee this.
 */

/** Virtual empty destructor. */
FawkesNetworkHub::~FawkesNetworkHub()
{
}

} // end namespace fawkes
