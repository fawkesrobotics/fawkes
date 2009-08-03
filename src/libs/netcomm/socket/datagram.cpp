
/***************************************************************************
 *  datagram.cpp - Fawkes datagram socket (TCP)
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

#include <netcomm/socket/datagram.h>

#include <sys/socket.h>

namespace fawkes {

/** @class DatagramSocket netcomm/socket/datagram.h
 * Datagram socket. A UDP socket on top of IP.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor.
 * @param timeout timeout, if 0 all operationsare blocking, otherwise it
 * is tried for timeout seconds.
 */
DatagramSocket::DatagramSocket(float timeout)
  : Socket(PF_INET, SOCK_DGRAM, 0, timeout)
{
}


/** Copy constructor.
 * @param datagram_socket socket to copy.
 */
DatagramSocket::DatagramSocket(DatagramSocket &datagram_socket)
  : Socket(datagram_socket)
{
}


/** Clone socket.
 * @return a copied instance of DatagramSocket.
 */
Socket *
DatagramSocket::clone()
{
  return new DatagramSocket(*this);
}

} // end namespace fawkes
