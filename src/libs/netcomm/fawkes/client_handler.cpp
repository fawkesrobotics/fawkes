
/***************************************************************************
 *  client_handler.cpp - handle incoming msgs from FawkesNetworkClientThread
 *
 *  Created: Thu Jun 14 17:18:39 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/fawkes/client_handler.h>

/** @class FawkesNetworkClientHandler <netcomm/fawkes/client_handler.h>
 * Message handler for FawkesNetworkClient.
 * This interface is used by the FawkesNetworkClient to handle incoming messages.
 * @ingroup NetComm
 * @author Tim Niemueller
 *
 * @fn virtual void FawkesNetworkClientHandler::deregistered() = 0
 * This handler has been deregistered.
 * This is called when this handler is deregistered from the
 * FawkesNetworkClient. Sometimes you may not want to allow this and post
 * a big fat warning into the log.
 *
 * @fn virtual void FawkesNetworkClientHandler::inboundReceived(FawkesNetworkMessage *m) = 0
 * Called for incoming messages.
 * This is called when an incoming message has been received. If this
 * method was called one or more times then the a previously carried out
 * wait(cid) call will continue.
 * @param m Message to handle
 *
 */

/** Empty virtual destructor. */
FawkesNetworkClientHandler::~FawkesNetworkClientHandler()
{
}
