
/***************************************************************************
 *  gossip.h - Robot Group Communication Aspect
 *
 *  Created: Fri Feb 28 11:13:50 2014
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

#ifndef __PLUGINS_GOSSIP_ASPECT_GOSSIP_H_
#define __PLUGINS_GOSSIP_ASPECT_GOSSIP_H_

#include <aspect/aspect.h>
#include <core/utils/refptr.h>

#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class GossipGroup;
class GossipAspectIniFin;

class GossipAspect : public virtual Aspect
{
 friend GossipAspectIniFin;

 public:
  GossipAspect(const char *group_name);
  virtual ~GossipAspect();

 protected:
  RefPtr<GossipGroup> gossip_group;

 private:
  const std::string GossipAspect_group_name_;
};

} // end namespace fawkes

#endif
