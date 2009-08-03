
/***************************************************************************
 *  handler.cpp - Fawkes network traffic handler
 *
 *  Created: Mon Nov 20 15:07:01 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/fawkes/handler.h>

namespace fawkes {

/** @class FawkesNetworkHandler handler.h <netcomm/fawkes/handler.h>
 * Network handler abstract base class.
 * This class shall be extended by threads that want to use the Fawkes
 * network connection.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 *
 *
 * @fn void FawkesNetworkHandler::handle_network_message(FawkesNetworkMessage *msg) = 0
 * Called for incoming messages that are addressed to the correct component ID.
 * Note that this message should be processed really really fast! A good idea is to enqueue
 * the message in an inbound queue (remember to ref() it!) and then process it in the next
 * run of loop() or wakeup a processing thread.
 * @param msg message to handle. If you want to keep this message you have to ref() it!
 * It is guaranteed that the message will not be erased during the handleNetworkMessage()
 * run, but afterwards no guarantee is made. So if you want to store the message internally
 * for example for later processing you have to reference the message.
 *
 * @fn void FawkesNetworkHandler::client_connected(unsigned int clid) = 0
 * Called when a new client connected. If any actions need to be taken on your side this
 * is the place to do it.
 * @param clid client ID of new client
 *
 * @fn void FawkesNetworkHandler::client_disconnected(unsigned int clid) = 0
 * Called when a client disconnected. If any actions need to be taken on your side this
 * is the place to do it. Note that you cannot send any further messages to this client!
 * @param clid client ID of disconnected client
 *
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

} // end namespace fawkes
