
/***************************************************************************
 *  skel_if_observer.h - Skeleton interface observer
 *
 *  Created: Sat Apr 02 18:14:37 2011 (RoboCup German Open 2011, Magdeburg)
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_OPENNI_UTILS_TYPES_H_
#define __PLUGINS_OPENNI_UTILS_TYPES_H_

#include <map>
#include <string>

namespace fawkes {
  class HumanSkeletonInterface;
  class HumanSkeletonProjectionInterface;
  class ObjectPositionInterface;

  namespace openni {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** User info to pass to draw_skeletons(). */
typedef struct {
  fawkes::HumanSkeletonInterface            *skel_if;	/**< Skeleton interface. */
  fawkes::HumanSkeletonProjectionInterface  *proj_if;	/**< Projection interface. */
} UserInfo;

/** Map from name to user info. */
typedef std::map<std::string, UserInfo>  UserMap;


/** Hand info to pass to draw_skeletons(). */
typedef struct {
  fawkes::ObjectPositionInterface           *hand_if;	/**< Hand pos interface. */
} HandInfo;

/** Map from name to hand info. */
typedef std::map<std::string, HandInfo>  HandMap;

} // end namespace fawkes::openni
} // end namespace fawkes

#endif
