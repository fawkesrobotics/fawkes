
/***************************************************************************
 *  datagram_broadcast.h - Fawkes broadcast datagram socket (UDP)
 *
 *  Created: Sun Nov 26 17:23:25 2006
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

#ifndef __NETCOMM_SOCKET_DATAGRAM_BROADCAST_H_
#define __NETCOMM_SOCKET_DATAGRAM_BROADCAST_H_

#include <netcomm/socket/socket.h>
#include <netinet/in.h>

namespace fawkes {

class BroadcastDatagramSocket : public Socket
{
 public:
  BroadcastDatagramSocket(const char *broadcast_addr_s, unsigned short port,
			  float timeout = 0.f);
  BroadcastDatagramSocket(BroadcastDatagramSocket &s);
  virtual ~BroadcastDatagramSocket();

  virtual Socket *  clone();

  virtual void bind();

  virtual void send(void *buf, unsigned int buf_len);

 private:
  struct ::sockaddr_in *broadcast_addr;

};

} // end namespace fawkes

#endif
