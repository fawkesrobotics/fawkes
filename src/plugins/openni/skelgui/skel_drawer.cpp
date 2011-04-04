
/***************************************************************************
 *  skel_drawer.cpp - Skeleton Visualization GUI: skeleton drawer
 *
 *  Created: Wed Mar 02 11:36:43 2011
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

#include "skel_drawer.h"

#include <utils/math/angle.h>

#include <plugins/openni/utils/colors.h>


#include <cstring>
#include <cstdio>
#include <GL/glut.h>

using namespace fawkes;
using namespace fawkes::openni;

/** @class SkelGuiSkeletonDrawer "skel_drawer.h"
 * Draw body skeleton using OpenGL.
 * This class draws the limbs as read from the user interfaces.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param users map of users shared with interface observer
 * @param hands map of hands shared with interface observer
 */
SkelGuiSkeletonDrawer::SkelGuiSkeletonDrawer(UserMap &users, HandMap &hands)
  : __users(users), __hands(hands)
{
  __print_state = PRINT_ID_STATE;
}

void
SkelGuiSkeletonDrawer::print_string(void *font, char *str)
{
  const int l = strlen(str);
  for(int i = 0; i < l; ++i)  glutBitmapCharacter(font, *str++);
}

void
SkelGuiSkeletonDrawer::draw_limb(float *proj1, float conf1,
				 float *proj2, float conf2)
{
  if (conf1 < 0.5 || conf2 < 0.5)  return;

  glVertex3i(proj1[0], proj1[1], 0);
  glVertex3i(proj2[0], proj2[1], 0);
}

#define DRAW_LIMB(user, joint1, joint2)					\
  draw_limb(user.proj_if->proj_##joint1(),				\
            user.skel_if->pos_##joint1##_confidence(),			\
	    user.proj_if->proj_##joint2(),				\
	    user.skel_if->pos_##joint2##_confidence());

void
SkelGuiSkeletonDrawer::draw_user(UserInfo &user)
{
  if (user.skel_if->state() != HumanSkeletonInterface::STATE_TRACKING)  return;

  DRAW_LIMB(user, head, neck);
  
  DRAW_LIMB(user, neck, left_shoulder);
  DRAW_LIMB(user, left_shoulder, left_elbow);
  DRAW_LIMB(user, left_elbow, left_hand);
  
  DRAW_LIMB(user, neck, right_shoulder);
  DRAW_LIMB(user, right_shoulder, right_elbow);
  DRAW_LIMB(user, right_elbow, right_hand);

  DRAW_LIMB(user, left_shoulder, torso);
  DRAW_LIMB(user, right_shoulder, torso);

  DRAW_LIMB(user, torso, left_hip);
  DRAW_LIMB(user, left_hip, left_knee);
  DRAW_LIMB(user, left_knee, left_foot);

  DRAW_LIMB(user, torso, right_hip);
  DRAW_LIMB(user, right_hip, right_knee);
  DRAW_LIMB(user, right_knee, right_foot);

  DRAW_LIMB(user, left_hip, right_hip);

}

void
SkelGuiSkeletonDrawer::draw_circle(unsigned int id, float *proj, float radius)
{
  glBegin(GL_LINE_LOOP);
  glVertex2f(proj[0], proj[1]);
  glColor4f(1 - USER_COLORS[id % NUM_USER_COLORS][0],
	    1 - USER_COLORS[id % NUM_USER_COLORS][1],
	    1 - USER_COLORS[id % NUM_USER_COLORS][2],
	    1);
  for (int i=0; i < 360; ++i) {
    float rad = deg2rad(i);;
    glVertex2f( proj[0] + cos(rad) * radius, proj[1] + sin(rad) * radius);
  }
  glColor4f(1, 1, 1, 1);
  glEnd();
}


/** Draw skeletons. */
void
SkelGuiSkeletonDrawer::draw()
{
  char label[50] = "";
  for (UserMap::iterator i = __users.begin(); i != __users.end(); ++i) {
    if (i->second.skel_if->state() != HumanSkeletonInterface::STATE_INVALID) {
      if (__print_state != PRINT_NONE) {
	memset(label, 0, sizeof(label));
	if (__print_state == PRINT_ID) {
	  sprintf(label, "%s", i->first.c_str());
	}
	else if (i->second.skel_if->state() == HumanSkeletonInterface::STATE_TRACKING)
	{
	  sprintf(label, "%s - Tracking", i->first.c_str());
	} else if (i->second.skel_if->state() == HumanSkeletonInterface::STATE_CALIBRATING)
	{
	  sprintf(label, "%s - Calibrating...", i->first.c_str());
	} else {
	sprintf(label, "%s - Looking for pose", i->first.c_str());
	}

	glColor4f(1 - USER_COLORS[i->second.skel_if->user_id() % NUM_USER_COLORS][0],
		  1 - USER_COLORS[i->second.skel_if->user_id() % NUM_USER_COLORS][1],
		  1 - USER_COLORS[i->second.skel_if->user_id() % NUM_USER_COLORS][2],
		  1);
      
	glRasterPos2i(i->second.proj_if->proj_com(0), i->second.proj_if->proj_com(1));
	print_string(GLUT_BITMAP_HELVETICA_18, label);
      }

      glBegin(GL_LINES);
      glColor4f(1 - USER_COLORS[i->second.skel_if->user_id() % NUM_USER_COLORS][0],
		1 - USER_COLORS[i->second.skel_if->user_id() % NUM_USER_COLORS][1],
		1 - USER_COLORS[i->second.skel_if->user_id() % NUM_USER_COLORS][2],
		1);

      draw_user(i->second);
      glEnd();
    }
  }

  glEnable(GL_LINE_SMOOTH);
  glLineWidth(4);
  for (HandMap::iterator i = __hands.begin(); i != __hands.end(); ++i) {
    if (i->second.hand_if->is_visible()) {
      float proj[2] = {i->second.hand_if->world_x(), i->second.hand_if->world_y()};
      draw_circle(i->second.hand_if->world_z(), proj, 10);
    }
  }
  glLineWidth(1.);
  glDisable(GL_LINE_SMOOTH);
}

/** Toggle the printing state.
 * This toggles through the printing state in the order PRINT_NONE,
 * PRINT_ID_STATE, and PRINT_ID.
 */
void
SkelGuiSkeletonDrawer::toggle_print_state()
{
  switch (__print_state) {
  case PRINT_NONE:      __print_state = PRINT_ID_STATE; break;
  case PRINT_ID_STATE:  __print_state = PRINT_ID;       break;
  case PRINT_ID:        __print_state = PRINT_NONE;     break;
  }
}


/** Set print state.
 * @param state new print state
 */
void
SkelGuiSkeletonDrawer::set_print_state(SkelGuiSkeletonDrawer::PrintState state)
{
  glBegin(GL_LINE_LOOP);

  __print_state = state;
  glEnd();
}
