
/***************************************************************************
 *  fuse_message_queue.h - Fuse network message queue
 *
 *  Created: Wed Nov 14 13:35:35 2007
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_MESSAGE_QUEUE_H_
#define __FIREVISION_FVUTILS_NET_FUSE_MESSAGE_QUEUE_H_

#include <core/utils/lock_queue.h>
#include <fvutils/net/fuse_message.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FuseNetworkMessageQueue <fvutils/net/fuse_message_queue.h>
 * A LockQueue of FuseNetworkMessage to hold messages in inbound and
 * outbound queues.
 * @author Tim Niemueller
 */
class FuseNetworkMessageQueue : public fawkes::LockQueue<FuseNetworkMessage *>
{
};

} // end namespace firevision

#endif
