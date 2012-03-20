
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

#include <plugins/openni/utils/types.h>

#include <interfaces/HumanSkeletonInterface.h>
#include <interfaces/HumanSkeletonProjectionInterface.h>
#include <interfaces/ObjectPositionInterface.h>

class SkelGuiSkeletonDrawer
{
 public:
  /** Print state enum. */
  typedef enum {
    PRINT_NONE,		/**< Print neither ID nor state */
    PRINT_ID,		/**< Print only ID */
    PRINT_ID_STATE	/**< Print ID and state */
  } PrintState;

  SkelGuiSkeletonDrawer(fawkes::openni::UserMap &users,
			fawkes::openni::HandMap &hands);

  void draw();

  void toggle_print_state();
  void set_print_state(PrintState state);

 private:
  void print_string(void *font, char *str);
  void draw_limb(float *proj1, float conf1, float *proj2, float conf2);
  void draw_user(fawkes::openni::UserInfo &user);
  void draw_circle(unsigned int id, float *proj, float radius);

 private:
  fawkes::openni::UserMap  &__users;
  fawkes::openni::HandMap  &__hands;

  PrintState __print_state;
};

#endif
