
/***************************************************************************
 *  hub.h - Fawkes network hub
 *
 *  Created: Mon May 07 19:06:30 2007
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

#ifndef __NETCOMM_FAWKES_HUB_H_
#define __NETCOMM_FAWKES_HUB_H_

class FawkesNetworkMessage;
class FawkesNetworkHandler;
class FawkesNetworkMessageContent;

class FawkesNetworkHub
{
 public:
  virtual ~FawkesNetworkHub();

  virtual void broadcast(FawkesNetworkMessage *msg)                        = 0;

  virtual void broadcast(unsigned short int component_id,
			 unsigned short int msg_id,
			 void *payload, unsigned int payload_size)         = 0;

  virtual void broadcast(unsigned short int component_id,
			 unsigned short int msg_id)                        = 0;


  virtual void send(FawkesNetworkMessage *msg)                             = 0;

  virtual void send(unsigned int to_clid,
		    unsigned short int component_id,
		    unsigned short int msg_id)                             = 0;

  virtual void send(unsigned int to_clid,
		    unsigned short int component_id,
		    unsigned short int msg_id,
		    void *payload, unsigned int payload_size)              = 0;

  virtual void send(unsigned int to_clid,
		    unsigned short int component_id,
		    unsigned short int msg_id,
		    FawkesNetworkMessageContent *content)                  = 0;

  virtual void add_handler(FawkesNetworkHandler *handler)                  = 0;
  virtual void remove_handler(FawkesNetworkHandler *handler)               = 0;


  virtual void force_send()                                                = 0;

};

#endif
