
/***************************************************************************
 *  client_handler.h - handle incoming msgs from FawkesNetworkClientThread
 *
 *  Created: Mon Jan 08 14:18:10 2007
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

#ifndef __NETCOMM_FAWKES_CLIENT_HANDLER_H_
#define __NETCOMM_FAWKES_CLIENT_HANDLER_H_

namespace fawkes {

class FawkesNetworkMessage;

class FawkesNetworkClientHandler
{
 public:
  virtual ~FawkesNetworkClientHandler();

  virtual void deregistered(unsigned int id) throw()                      = 0;
  virtual void inbound_received(FawkesNetworkMessage *m,
				unsigned int id) throw()                  = 0;
  virtual void connection_died(unsigned int id) throw()                   = 0;
  virtual void connection_established(unsigned int id) throw()            = 0;

};

} // end namespace fawkes

#endif
