
/***************************************************************************
 *  datagram_multicast.h - Fawkes multicast datagram socket (UDP)
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

#ifndef _NETCOMM_SOCKET_DATAGRAM_MULTICAST_H_
#define _NETCOMM_SOCKET_DATAGRAM_MULTICAST_H_

#include <netcomm/socket/socket.h>
#include <netinet/in.h>

namespace fawkes {

class MulticastDatagramSocket : public Socket
{
public:
	MulticastDatagramSocket(AddrType       addr_type,
	                        const char    *multicast_addr_s,
	                        unsigned short port,
	                        float          timeout = 0.f);
	MulticastDatagramSocket(MulticastDatagramSocket &s);
	virtual ~MulticastDatagramSocket();

	MulticastDatagramSocket &operator=(MulticastDatagramSocket &s);

	virtual Socket *clone();

	virtual void bind();
	virtual void bind(const unsigned short int port);
	virtual void bind(const unsigned short int port, const char *hostname);

	virtual void send(void *buf, size_t buf_len);
	virtual void
	send(void *buf, size_t buf_len, const struct sockaddr *to_addr, socklen_t addr_len)
	{
		send(buf, buf_len);
	}

	void set_loop(bool loop);
	void set_ttl(int ttl);

private:
	struct ::sockaddr_in *multicast_addr;
};

} // end namespace fawkes

#endif
