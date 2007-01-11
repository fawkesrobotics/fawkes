
/***************************************************************************
 *  handler.h - Fawkes network traffic handler
 *
 *  Created: Mon Nov 20 14:36:06 2006
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

#ifndef __NETCOMM_FAWKES_HANDLER_H_
#define __NETCOMM_FAWKES_HANDLER_H_

// needed anyway, not worth a foreward declaration
#include <netcomm/fawkes/emitter.h>
#include <netcomm/fawkes/message.h>

class FawkesNetworkHandler
{
 public:
  FawkesNetworkHandler(unsigned short int id);
  virtual ~FawkesNetworkHandler();

  unsigned short int id() const;

  void setEmitter(FawkesNetworkEmitter *emitter);

  void broadcast(FawkesNetworkMessage *msg);
  void broadcast(unsigned short int msg_id,
		 void *payload, unsigned int payload_size);
  void broadcast(unsigned short int msg_id);

  void send(FawkesNetworkMessage *msg);
  void send(unsigned int to_clid, unsigned short int msg_id);
  void send(unsigned int to_clid, unsigned short int msg_id,
	    void *payload, unsigned int payload_size);


  virtual void handleNetworkMessage(FawkesNetworkMessage *msg)                = 0;
  virtual void clientConnected(unsigned int clid)                             = 0;
  virtual void clientDisconnected(unsigned int clid)                          = 0;
  virtual void processAfterLoop()                                             = 0;

 protected:
  FawkesNetworkEmitter *emitter;

 private:
  unsigned short int _id;

};

#endif
