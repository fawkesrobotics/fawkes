
/***************************************************************************
 *  fuse_transceiver.h - Fuse transceiver
 *
 *  Created: Wed Nov 14 13:23:56 2007
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_TRANSCEIVER_H_
#define __FIREVISION_FVUTILS_NET_FUSE_TRANSCEIVER_H_

#include <core/exception.h>

class StreamSocket;
class FuseNetworkMessageQueue;

class FuseNetworkTransceiver
{
 public:
  static void send(StreamSocket *s, FuseNetworkMessageQueue *msgq);
  static void recv(StreamSocket *s, FuseNetworkMessageQueue *msgq,
		   unsigned int max_num_msgs = 8);
};

#endif
