
/***************************************************************************
 *  datagram.h - Fawkes datagram socket (UDP)
 *
 *  Created: Mon Nov 13 19:06:24 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_SOCKET_DATAGRAM_H_
#define __NETCOMM_SOCKET_DATAGRAM_H_

#include <netcomm/socket/socket.h>

class DatagramSocket : public Socket
{
 public:
  DatagramSocket(float timeout = 0.f);
  DatagramSocket(DatagramSocket &s);

  virtual Socket *  clone();

 protected:
};

#endif
