
/***************************************************************************
 *  gossip_group_manager.cpp - Fawkes Gossip group manager
 *
 *  Created: Fri Feb 28 16:55:24 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include <plugins/gossip/gossip/gossip_group_manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class GossipGroupManager <plugins/gossip/gossip/gossip_group_manager.h>
 * Abstract class for a Gossip group manager.
 * @author Tim Niemueller
 *
 * @fn fawkes::RefPtr<fawkes::GossipGroup> GossipGroupManager::join_group(const char *name) = 0
 * Join a gossip group.
 * @param name the name of the group to join
 * @return a shared object to communicate with the group.
 *
 * @fn void GossipGroupManager::leave_group(fawkes::RefPtr<fawkes::GossipGroup> &group) = 0
 * Leave a gossip group.
 * @param group the gossip group to leave, the handle becomes invalid after this call.
 */

/** Virtual empty destructor. */
GossipGroupManager::~GossipGroupManager()
{
}


} // end namespace fawkes
