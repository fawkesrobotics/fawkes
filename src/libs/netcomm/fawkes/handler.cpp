
/***************************************************************************
 *  handler.cpp - Fawkes network traffic handler
 *
 *  Created: Mon Nov 20 15:07:01 2006
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

#include <netcomm/fawkes/handler.h>

/** @class FawkesNetworkHandler netcomm/fawkes/handler.h
 * Network handler abstract base class.
 * This class shall be extended by threads that want to use the Fawkes
 * network connection.
 *
 * @author Tim Niemueller
 */

/** @var FawkesNetworkHandler::emitter
 * Emitter to send messages with.
 */
/** @fn void FawkesNetworkHandler::handleNetworkMessage(FawkesNetworkMessage *msg) = 0
 * Called for incoming messages that are addressed to the correct component ID.
 * @param msg message to handle. If you want to keep this message you have to ref() it!
 * It is guaranteed that the message will not be erased during the handleNetworkMessage()
 * run, but afterwards no guarantee is made. So if you want to store the message internally
 * for example for later processing you have to reference the message.
 */
/** @fn void FawkesNetworkHandler::clientConnected(unsigned int clid) = 0
 * Called when a new client connected. If any actions need to be taken on your side this
 * is the place to do it.
 * @param clid client ID of new client
 */

/** @fn void FawkesNetworkHandler::clientDisconnected(unsigned int clid) = 0
 * Called when a client disconnected. If any actions need to be taken on your side this
 * is the place to do it. Note that you cannot send any further messages to this client!
 * @param clid client ID of disconnected client
 */

/** Constructor.
 * @param id the component ID this handlers wants to handle.
 */
FawkesNetworkHandler::FawkesNetworkHandler(unsigned short int id)
{
  _id = id;
}


/** Destructor. */
FawkesNetworkHandler::~FawkesNetworkHandler()
{
}


/** Get the component ID for this handler.
 * @return component ID
 */
unsigned short int
FawkesNetworkHandler::id() const
{
  return _id;
}


/** Set emitter.
 * This method has to be called before the handler is actually used. If used
 * inside Fawkes this is guaranteed by the thread initializer.
 * @param emitter Fawkes emitter
 */
void
FawkesNetworkHandler::setEmitter(FawkesNetworkEmitter *emitter)
{
  this->emitter = emitter;
}


/** Broadcast a message.
 * Copied through to the emitter.
 * @param msg message to broadcast
 * @see FawkesNetworkEmitter::broadcast()
 */
void
FawkesNetworkHandler::broadcast(FawkesNetworkMessage *msg)
{
  emitter->broadcast(msg);
}


/** Broadcast a message.
 * A FawkesNetworkMessage is created and broacasted via the emitter.
 * @param msg_id message type id
 * @param payload payload buffer
 * @param payload_size size of payload buffer
 * @see FawkesNetworkEmitter::broadcast()
 */
void
FawkesNetworkHandler::broadcast(unsigned short int msg_id,
				void *payload, unsigned int payload_size)
{
  FawkesNetworkMessage *m = new FawkesNetworkMessage(_id, msg_id, payload, payload_size);
  emitter->broadcast(m);
  m->unref();
}


/** Send a message.
 * Copied through to the emitter.
 * @param msg message to broadcast
 * @see FawkesNetworkEmitter::send()
 */
void
FawkesNetworkHandler::send(FawkesNetworkMessage *msg)
{
  emitter->send(msg);
}


/** Send a message.
 * A FawkesNetworkMessage is created and sent via the emitter.
 * @param to_clid client ID of recipient
 * @param msg_id message type id
 * @param payload payload buffer
 * @param payload_size size of payload buffer
 * @see FawkesNetworkEmitter::broadcast()
 */
void
FawkesNetworkHandler::send(unsigned int to_clid, unsigned short int msg_id,
			   void *payload, unsigned int payload_size)
{
  FawkesNetworkMessage *m = new FawkesNetworkMessage(to_clid, _id, msg_id, payload, payload_size);
  emitter->send(m);
  m->unref();
}
