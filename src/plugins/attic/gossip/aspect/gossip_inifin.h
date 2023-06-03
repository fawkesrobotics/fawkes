
/***************************************************************************
 *  gossip_inifin.h - Fawkes GossipAspect initializer/finalizer
 *
 *  Created: Fri Feb 28 16:51:32 2014
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

#ifndef _PLUGINS_GOSSIP_ASPECT_GOSSIP_INIFIN_H_
#define _PLUGINS_GOSSIP_ASPECT_GOSSIP_INIFIN_H_

#include <aspect/inifins/inifin.h>
#include <plugins/gossip/aspect/gossip.h>

namespace fawkes {

class GossipGroupManager;

class GossipAspectIniFin : public AspectIniFin
{
public:
	GossipAspectIniFin();
	~GossipAspectIniFin();

	virtual void init(Thread *thread);
	virtual void finalize(Thread *thread);

	void set_manager(GossipGroupManager *gossip_group_mgr);

private:
	GossipGroupManager *gossip_group_mgr_;
};

} // end namespace fawkes

#endif
