
/***************************************************************************
 *  fuse_transceiver.h - Fuse transceiver
 *
 *  Created: Wed Nov 14 13:23:56 2007
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_TRANSCEIVER_H_
#define __FIREVISION_FVUTILS_NET_FUSE_TRANSCEIVER_H_

#include <core/exception.h>

namespace fawkes {
  class StreamSocket;
}
namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FuseNetworkMessageQueue;

class FuseNetworkTransceiver
{
 public:
  static void send(fawkes::StreamSocket *s, FuseNetworkMessageQueue *msgq);
  static void recv(fawkes::StreamSocket *s, FuseNetworkMessageQueue *msgq,
		   unsigned int max_num_msgs = 8);
};

} // end namespace firevision

#endif
