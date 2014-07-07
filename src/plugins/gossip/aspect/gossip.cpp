
/***************************************************************************
 *  gossip.cpp - Robot Group Communication Aspect
 *
 *  Created: Fri Feb 28 16:47:24 2014
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

#include <plugins/gossip/aspect/gossip.h>
#include <plugins/gossip/gossip/gossip_group.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class GossipAspect <plugins/gossip/aspect/gossip.h>
 * Thread aspect to communicate with a group of robots.
 * Give this aspect to your thread to get access to a communication
 * group.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @var fawkes:RefPtr<GossipGroup> GossipAspect::gossip_group
 * Gossip group to communicate with other robots.
 */

/** Constructor.
 * @param group_name Gossip group to join and communicate with.
 */
GossipAspect::GossipAspect(const char *group_name)
  : GossipAspect_group_name_(group_name)
{
  add_aspect("GossipAspect");
}


/** Virtual empty destructor. */
GossipAspect::~GossipAspect()
{
}


} // end namespace fawkes
