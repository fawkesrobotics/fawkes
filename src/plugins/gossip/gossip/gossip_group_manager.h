
/***************************************************************************
 *  gossip_group_manager.h - Fawkes Gossip group manager
 *
 *  Created: Fri Feb 28 16:54:34 2014
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

#ifndef __PLUGINS_GOSSIP_GOSSIP_GOSSIP_GROUP_MANAGER_H_
#define __PLUGINS_GOSSIP_GOSSIP_GOSSIP_GROUP_MANAGER_H_

#include <core/utils/refptr.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class GossipGroup;

class GossipGroupManager
{
 public:
  virtual ~GossipGroupManager();

  virtual RefPtr<GossipGroup> join_group(const char *name) = 0;
  virtual void leave_group(RefPtr<GossipGroup> &group) = 0;
};

} // end namespace fawkes

#endif

