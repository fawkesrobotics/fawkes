
/***************************************************************************
 *  handler.h - Fawkes network traffic handler
 *
 *  Created: Mon Nov 20 14:36:06 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_FAWKES_HANDLER_H_
#define __NETCOMM_FAWKES_HANDLER_H_

#include <netcomm/fawkes/message.h>

namespace fawkes {

class FawkesNetworkHandler
{
 public:
  FawkesNetworkHandler(unsigned short int id);
  virtual ~FawkesNetworkHandler();

  unsigned short int id() const;

  virtual void handle_network_message(FawkesNetworkMessage *msg)           = 0;
  virtual void client_connected(unsigned int clid)                         = 0;
  virtual void client_disconnected(unsigned int clid)                      = 0;

 private:
  unsigned short int _id;

};

} // end namespace fawkes

#endif
