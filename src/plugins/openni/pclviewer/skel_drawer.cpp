
/***************************************************************************
 *  skel_drawer.cpp - OpenNI Visualization: 3D skeleton drawer
 *
 *  Created: Sat Apr 02 20:00:50 2011
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
#include <plugins/openni/utils/colors.h>

#include <utils/math/angle.h>

#include <cstring>
#include <cstdio>
#include <GL/glut.h>

using namespace fawkes;
using namespace fawkes::openni;

/** @class SkelGuiSkeletonDrawer3D "skel_drawer.h"
 * Draw body skeleton using OpenGL (3D).
 * This class draws the limbs as read from the user interfaces. This version
 * draws in 3D and does not use the 2D projection.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param users map of users shared with interface observer
 * @param hands map of hands shared with interface observer
 */
SkelGuiSkeletonDrawer3D::SkelGuiSkeletonDrawer3D(UserMap &users, HandMap &hands)
  : __users(users), __hands(hands)
{
  __print_state = PRINT_ID_STATE;
}

void
SkelGuiSkeletonDrawer3D::draw_limb(float *p1, float conf1,
				 float *p2, float conf2)
{
  if (conf1 < 0.5 || conf2 < 0.5)  return;

  //printf("Drawing from (%f,%f,%f) -> (%f,%f,%f)\n",
  //	 p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
  glVertex4f(p1[0], p1[1], p1[2], 1);
  glVertex4f(p2[0], p2[1], p2[2], 1);
}

#define DRAW_LIMB(user, joint1, joint2)					\
  draw_limb(user.skel_if->pos_##joint1(),				\
            user.skel_if->pos_##joint1##_confidence(),			\
	    user.skel_if->pos_##joint2(),				\
	    user.skel_if->pos_##joint2##_confidence());

void
SkelGuiSkeletonDrawer3D::draw_user(UserInfo &user)
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

/** Draw skeletons. */
void
SkelGuiSkeletonDrawer3D::draw()
{
  for (UserMap::iterator i = __users.begin(); i != __users.end(); ++i) {
    i->second.skel_if->read();
    if (i->second.skel_if->state() != HumanSkeletonInterface::STATE_INVALID) {
      glPointSize(10);
      glBegin(GL_POINTS);
      glColor4f(1 - USER_COLORS[i->second.skel_if->user_id() % NUM_USER_COLORS][0],
		1 - USER_COLORS[i->second.skel_if->user_id() % NUM_USER_COLORS][1],
		1 - USER_COLORS[i->second.skel_if->user_id() % NUM_USER_COLORS][2],
		1);
      float *com = i->second.skel_if->com();
      glVertex4f(com[0], com[1], com[2], 1.0);
      glEnd();
      glPointSize(1);

      glLineWidth(3);
      glBegin(GL_LINES);
      draw_user(i->second);
      glColor4f(1, 1, 1, 1);
      glEnd();
      glLineWidth(1);
    }
  }

  /*
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
  */
}

/** Toggle the printing state.
 * This toggles through the printing state in the order PRINT_NONE,
 * PRINT_ID_STATE, and PRINT_ID.
 */
void
SkelGuiSkeletonDrawer3D::toggle_print_state()
{
  switch (__print_state) {
  case PRINT_NONE:      __print_state = PRINT_ID_STATE; break;
  case PRINT_ID_STATE:  __print_state = PRINT_ID;       break;
  case PRINT_ID:        __print_state = PRINT_NONE;     break;
  }
}


void
SkelGuiSkeletonDrawer3D::draw_circle(unsigned int id, float *p, float radius)
{
  glBegin(GL_LINE_LOOP);
  glVertex3f(p[0], p[1], p[2]);
  glColor4f(1 - USER_COLORS[id % NUM_USER_COLORS][0],
	    1 - USER_COLORS[id % NUM_USER_COLORS][1],
	    1 - USER_COLORS[id % NUM_USER_COLORS][2],
	    1);
  for (int i=0; i < 360; ++i) {
    float rad = deg2rad(i);;
    glVertex3f( p[0] + cos(rad) * radius, p[1] + sin(rad) * radius, p[2]);
  }
  glColor4f(1, 1, 1, 1);
  glEnd();
}


/** Set print state.
 * @param state new print state
 */
void
SkelGuiSkeletonDrawer3D::set_print_state(SkelGuiSkeletonDrawer3D::PrintState state)
{
  __print_state = state;
}
