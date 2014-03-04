
/***************************************************************************
 *  gossip_group.cpp - Robot Group Communication - Gossip Group
 *
 *  Created: Tue Mar 04 11:00:11 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include <plugins/gossip/gossip/gossip_group.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class GossipGroup <plugins/gossip/gossip/gossip_group.h>
 * Gossip group communication handler.
 * The group communication handler cares about joining groups and
 * sending and receiving data.
 * @author Tim Niemueller
 */

/** Constructor.
 */
GossipGroup::GossipGroup()
{
}


/** Destructor. */
GossipGroup::~GossipGroup()
{
}


/** Send a message.
 * @param peer peer to send message to
 * @param m message to send
 */
void
GossipGroup::send(std::string &peer,
		  google::protobuf::Message &m)
{
}


/** Broadcast a message to all peers in the group.
 * @param m message to send
 */
void
GossipGroup::broadcast(google::protobuf::Message &m)
{
}


} // end namespace fawkes
