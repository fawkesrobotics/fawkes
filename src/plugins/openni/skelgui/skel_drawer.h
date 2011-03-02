
/***************************************************************************
 *  skel_drawer.h - Skeleton Visualization GUI: skeleton drawer
 *
 *  Created: Wed Mar 02 11:33:29 2011
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

#ifndef __PLUGINS_OPENNI_SKELGUI_SKEL_DRAWER_H_
#define __PLUGINS_OPENNI_SKELGUI_SKEL_DRAWER_H_

#include <interfaces/HumanSkeletonInterface.h>
#include <interfaces/HumanSkeletonProjectionInterface.h>

#include <map>
#include <string>

/** User info to pass to draw_skeletons(). */
typedef struct {
  fawkes::HumanSkeletonInterface            *skel_if;	/**< Skeleton interface. */
  fawkes::HumanSkeletonProjectionInterface  *proj_if;	/**< Projection interface. */
} UserInfo;

typedef std::map<std::string, UserInfo>  UserMap;

void draw_skeletons(UserMap &users, unsigned int x_res, unsigned int y_res);

#endif
