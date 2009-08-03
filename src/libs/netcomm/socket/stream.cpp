
/***************************************************************************
 *  stream.cpp - Fawkes stream socket (TCP)
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

#include <netcomm/socket/stream.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <errno.h>

namespace fawkes {


/** @class StreamSocket netcomm/socket/stream.h
 * TCP stream socket over IP.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor.
 * @param timeout timeout, if 0 all operationsare blocking, otherwise it
 * is tried for timeout seconds.
 */
StreamSocket::StreamSocket(float timeout)
  : Socket(PF_INET, SOCK_STREAM, 0, timeout)
{
}


/** Copy constructor.
 * @param stream_socket socket to copy.
 */
StreamSocket::StreamSocket(StreamSocket &stream_socket)
  : Socket(stream_socket)
{
}


/** Clone socket.
 * @return a copied instance of StreamSocket.
 */
Socket *
StreamSocket::clone()
{
  return new StreamSocket(*this);
}


/** Check if Nalge algorithm is disabled.
 * This checks the TCP_NODELAY option on the socket. If it is set then the
 * Nagle algorithm is disabled and all data is send out immediately.
 * @return true, if nodelay is enabled and thus the Nagle algorithm disabled,
 * false otherwise
 */
bool
StreamSocket::nodelay()
{
  int val = 0;
 socklen_t val_len = sizeof(val);
  if ( getsockopt(sock_fd, IPPROTO_TCP, TCP_NODELAY, &val, &val_len) == -1 ) {
    throw SocketException("StreamSocket::nodelay: getsockopt failed", errno);
  }
  return (val == 1);
}


/** Enable or disable Nagle algorithm.
 * @param nodelay true to disable Nagle algorithm, false to enable it
 * @see nodelay()
 */
void
StreamSocket::set_nodelay(bool nodelay)
{
  int val = (nodelay ? 1 : 0);
  socklen_t val_len = sizeof(val);
  if ( setsockopt(sock_fd, IPPROTO_TCP, TCP_NODELAY, &val, val_len) == -1 ) {
    throw SocketException("StreamSocket::set_nodelay: setsockopt failed", errno);
  }
}

} // end namespace fawkes
