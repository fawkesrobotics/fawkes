
/***************************************************************************
 *  emitter.h - Fawkes network emitter interface
 *
 *  Created: Mon Nov 20 15:15:58 2006
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

#ifndef __NETCOMM_FAWKES_EMITTER_H_
#define __NETCOMM_FAWKES_EMITTER_H_

class FawkesNetworkMessage;

/** @class FawkesNetworkEmitter netcomm/fawkes/emitter.h
 * Fawkes Network Emitter interface.
 * This interface is implemented by owner of the network connection to send
 * data via the network.
 */

class FawkesNetworkEmitter
{
 public:
  /** Virtual empty destructor. */
  virtual ~FawkesNetworkEmitter() {}

  /** Method to broadcast a message to all connected clients.
   * This method shall be implemented thus that the message is sent to all
   * connected clients.
   * @param msg message to send.
   */
  virtual void broadcast(FawkesNetworkMessage *msg)                        = 0;

  /** Method to send a message to a specific client.
   * The client ID provided in the message is used to determine the correct
   * recipient. If no client is connected for the given client ID the message
   * shall be silently ignored.
   * @param msg message to send
   */
  virtual void send(FawkesNetworkMessage *msg)                             = 0;
  
};

#endif
