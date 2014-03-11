
/***************************************************************************
 *  gossip_inifin.cpp - Fawkes GossipAspect initializer/finalizer
 *
 *  Created: Sat Jun 16 14:34:27 2012
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include <plugins/gossip/aspect/gossip_inifin.h>
#include <plugins/gossip/gossip/gossip_group_manager.h>
#include <plugins/gossip/gossip/gossip_group.h>
#include <core/threading/thread_finalizer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class GossipAspectIniFin <plugins/gossip/aspect/gossip_inifin.h>
 * GossipAspect initializer/finalizer.
 * This initializer/finalizer will join and leave gossip groups on
 * behalf of threads with the GossipAspect.
 * @author Tim Niemueller
 */

/** Constructor. */
GossipAspectIniFin::GossipAspectIniFin()
  : AspectIniFin("GossipAspect")
{
}

/** Destructor. */
GossipAspectIniFin::~GossipAspectIniFin()
{
}



void
GossipAspectIniFin::init(Thread *thread)
{
  GossipAspect *gossip_thread;
  gossip_thread = dynamic_cast<GossipAspect *>(thread);
  if (gossip_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "GossipAspect, but RTTI says it "
					  "has not. ", thread->name());
  }
  
  RefPtr<GossipGroup> group =
    gossip_group_mgr_->join_group(gossip_thread->GossipAspect_group_name_);

  gossip_thread->gossip_group = group;
}

void
GossipAspectIniFin::finalize(Thread *thread)
{
  GossipAspect *gossip_thread;
  gossip_thread = dynamic_cast<GossipAspect *>(thread);
  if (gossip_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"GossipAspect, but RTTI says it "
					"has not. ", thread->name());
  }

  gossip_group_mgr_->leave_group(gossip_thread->gossip_group);
}



/** Set gossip group manger.
 * @param gossip_group_mgr Gossip group manager
 */
void
GossipAspectIniFin::set_manager(GossipGroupManager *gossip_group_mgr)
{
  gossip_group_mgr_ = gossip_group_mgr;
}

} // end namespace fawkes
