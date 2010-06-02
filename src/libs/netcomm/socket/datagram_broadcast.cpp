
/***************************************************************************
 *  datagram_broadcast.cpp - Fawkes datagram broadcast socket (UDP)
 *
 *  Created: Fri 02 Apr 2010 03:30:55 PM CEST
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *  Copyright  2010  Christoph Schwering
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

#include <netcomm/socket/datagram_broadcast.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <cstring>
#include <cerrno>

namespace fawkes {

/** @class BroadcastDatagramSocket netcomm/socket/datagram.h
 * Broadcast datagram socket.
 * An broadcast UDP socket on top of IP.
 *
 * @ingroup NetComm
 * @author Christoph Schwering
 */

/** Constructor.
 * @param broadcast_addr_s textual representation of the broadcast IP address
 * to use for broadcast communication. NOT a hostname!
 * @param port port
 * @param timeout timeout, if 0 all operationsare blocking, otherwise it
 * is tried for timeout seconds.
 */
BroadcastDatagramSocket::BroadcastDatagramSocket(const char *broadcast_addr_s,
						 unsigned short port,
						 float timeout)
  : Socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP, timeout)
{
  broadcast_addr = (struct ::sockaddr_in *)malloc(sizeof(struct ::sockaddr_in));

  struct in_addr a;
  if ( inet_aton(broadcast_addr_s, &a) == -1 ) {
    throw SocketException("Invalid address given");
  }
  broadcast_addr->sin_family = AF_INET;
  broadcast_addr->sin_addr.s_addr = a.s_addr;
  broadcast_addr->sin_port = htons(port);

#if 0
  //set_ttl(1);
  set_loop(false);
#endif
}


/** Destructor. */
BroadcastDatagramSocket::~BroadcastDatagramSocket()
{
  free(broadcast_addr);
}


/** Copy constructor.
 * @param datagram_socket socket to copy.
 */
BroadcastDatagramSocket::BroadcastDatagramSocket(BroadcastDatagramSocket &datagram_socket)
  : Socket(datagram_socket)
{
  broadcast_addr = (struct ::sockaddr_in *)malloc(sizeof(struct ::sockaddr_in));
  memcpy(broadcast_addr, datagram_socket.broadcast_addr, sizeof(struct ::sockaddr_in));
}


/** Bind socket.
 * This will make the socket listen for incoming traffic.
 */
void
BroadcastDatagramSocket::bind()
{
  int broadcast = 1;
  if ( setsockopt(sock_fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1) {
    throw SocketException("Could not set SO_BROADCAST", errno);
  }

  int reuse = 1;
  if ( setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) == -1) {
    throw SocketException("Could not set SO_REUSEADDR", errno);
  }

  struct ::sockaddr_in local;
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = INADDR_ANY;
  local.sin_port = broadcast_addr->sin_port;

  if (::bind(sock_fd, (struct ::sockaddr *) &local, sizeof(local)) < 0) {
    throw SocketException("Could not bind to port", errno);
  }
}


/** Clone socket.
 * @return a copied instance of BroadcastDatagramSocket.
 */
Socket *
BroadcastDatagramSocket::clone()
{
  return new BroadcastDatagramSocket(*this);
}


/** Send data.
 * This will send the given data to the broadcast address specified
 * in the constructor.
 * @param buf buffer to write
 * @param buf_len length of buffer, number of bytes to write to stream
 */
void
BroadcastDatagramSocket::send(void *buf, unsigned int buf_len)
{
  try {
    Socket::send(buf, buf_len, (struct ::sockaddr *)broadcast_addr, sizeof(struct ::sockaddr_in));
  } catch (SocketException &e) {
    e.append("BroadcastDatagramSocket::send(void*, unsigned int) failed");
    throw;
  }
}

} // end namespace fawkes
