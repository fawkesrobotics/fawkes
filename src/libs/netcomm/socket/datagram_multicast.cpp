
/***************************************************************************
 *  datagram_multicast.cpp - Fawkes datagram multicast socket (UDP)
 *
 *  Created: Fri Nov 10 10:02:54 2006 (on train to Google, Hamburg)
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

#include <netcomm/socket/datagram_multicast.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <cstring>
#include <cerrno>

namespace fawkes {

/** @class MulticastDatagramSocket netcomm/socket/datagram.h
 * Multicast datagram socket.
 * An multicast UDP socket on top of IP.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor.
 * @param multicast_addr_s textual representation of the multicast IP address
 * to use for multicast communication. NOT a hostname!
 * @param port port
 * @param timeout timeout, if 0 all operationsare blocking, otherwise it
 * is tried for timeout seconds.
 */
MulticastDatagramSocket::MulticastDatagramSocket(const char *multicast_addr_s,
						 unsigned short port,
						 float timeout)
  : Socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP, timeout)
{
  multicast_addr = (struct ::sockaddr_in *)malloc(sizeof(struct ::sockaddr_in));

  struct in_addr a;
  if ( inet_aton(multicast_addr_s, &a) == -1 ) {
    throw SocketException("Invalid address given");
  }
  multicast_addr->sin_family = AF_INET;
  multicast_addr->sin_addr.s_addr = a.s_addr;
  multicast_addr->sin_port = htons(port);

  //set_ttl(1);
  set_loop(false);
}


/** Destructor. */
MulticastDatagramSocket::~MulticastDatagramSocket()
{
  free(multicast_addr);
}


/** Copy constructor.
 * @param datagram_socket socket to copy.
 */
MulticastDatagramSocket::MulticastDatagramSocket(MulticastDatagramSocket &datagram_socket)
  : Socket(datagram_socket)
{
  multicast_addr = (struct ::sockaddr_in *)malloc(sizeof(struct ::sockaddr_in));
  memcpy(multicast_addr, datagram_socket.multicast_addr, sizeof(struct ::sockaddr_in));
}


/** Bind socket.
 * This will make the socket listen for incoming traffic. It will also add this host to
 * the appropriate multicast group.
 */
void
MulticastDatagramSocket::bind()
{
  int reuse = 1;
  if ( setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) == -1) {
    throw SocketException("Could not set SO_REUSEADDR", errno);
  }

  struct ip_mreq imr;
  imr.imr_multiaddr.s_addr = multicast_addr->sin_addr.s_addr;
  imr.imr_interface.s_addr = htonl( INADDR_ANY );
  if ( setsockopt(sock_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &imr, sizeof(imr)) == -1 ) {
    throw SocketException("Could not add multicast group membership", errno);
  }

  struct ::sockaddr_in local;
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = INADDR_ANY;
  local.sin_port = multicast_addr->sin_port;

  if (::bind(sock_fd, (struct ::sockaddr *) &local, sizeof(local)) < 0) {
    throw SocketException("Could not bind to port", errno);
  }
}


/** Clone socket.
 * @return a copied instance of MulticastDatagramSocket.
 */
Socket *
MulticastDatagramSocket::clone()
{
  return new MulticastDatagramSocket(*this);
}


/** Send data.
 * This will send the given data to the multicast address specified
 * in the constructor.
 * @param buf buffer to write
 * @param buf_len length of buffer, number of bytes to write to stream
 */
void
MulticastDatagramSocket::send(void *buf, unsigned int buf_len)
{
  try {
    Socket::send(buf, buf_len, (struct ::sockaddr *)multicast_addr, sizeof(struct ::sockaddr_in));
  } catch (SocketException &e) {
    e.append("MulticastDatagramSocket::send(void*, unsigned int) failed");
    throw;
  }
}


/** Set loopback of sent packets.
 * @param loop true to deliver sent packets to local sockets, false prevent delivering
 */
void
MulticastDatagramSocket::set_loop(bool loop)
{
  int l = (loop ? 1 : 0);
  if (setsockopt(sock_fd, IPPROTO_IP, IP_MULTICAST_LOOP, &l, sizeof(l)) == -1) {
    throw SocketException("MulticastDatagramSocket::set_loop: setsockopt failed", errno);
  }
}


/** Set multicast time-to-live (TTL)
 * @param ttl time-to-live
 */
void
MulticastDatagramSocket::set_ttl(int ttl)
{
  if ( ttl < 0 ) ttl = -ttl;
  if ( setsockopt( sock_fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl) ) == -1 ) {
    throw SocketException("MulticastDatagramSocket::set_ttl: setsockopt failed", errno);
  }
}

} // end namespace fawkes
