
/***************************************************************************
 *  client_handler.h - handle incoming msgs from FawkesNetworkClientThread
 *
 *  Created: Mon Jan 08 14:18:10 2007
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

#ifndef __NETCOMM_FAWKES_CLIENT_HANDLER_H_
#define __NETCOMM_FAWKES_CLIENT_HANDLER_H_

class FawkesNetworkMessage;

class FawkesNetworkClientHandler
{
 public:
  /** Virtual empty destructor */
  virtual ~FawkesNetworkClientHandler() {}

  /** This handler has been deregistered.
   * This is called when this handler is deregistered from the
   * FawkesNetworkClient. Sometimes you may not want to allow this and post
   * a big fat warning into the log.
   */
  virtual void deregistered()                                             = 0;

  /** Called for incoming messages.
   * This is called when an incoming message has been received. If this
   * method was called one or more times then the a previously carried out
   * wait(cid) call will continue.
   * @param m Message
   */
  virtual void inboundReceived(FawkesNetworkMessage *m)                   = 0;

};


#endif
