
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
  virtual ~FawkesNetworkClientHandler();

  virtual void deregistered() throw()                                     = 0;
  virtual void inbound_received(FawkesNetworkMessage *m) throw()          = 0;
  virtual void connection_died() throw()                                  = 0;
  virtual void connection_established() throw()                           = 0;

};


#endif
