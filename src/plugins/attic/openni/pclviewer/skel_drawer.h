
/***************************************************************************
 *  skel_drawer.h - OpenNI Visualization: 3D skeleton drawer
 *
 *  Created: Sat Apr 02 19:56:55 2011
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

#ifndef _PLUGINS_OPENNI_PCLVIEWER_SKEL_DRAWER_H_
#define _PLUGINS_OPENNI_PCLVIEWER_SKEL_DRAWER_H_

#include <interfaces/HumanSkeletonInterface.h>
#include <interfaces/HumanSkeletonProjectionInterface.h>
#include <interfaces/ObjectPositionInterface.h>
#include <plugins/openni/utils/types.h>

class SkelGuiSkeletonDrawer3D
{
public:
	/** Print state enum. */
	typedef enum {
		PRINT_NONE,    /**< Print neither ID nor state */
		PRINT_ID,      /**< Print only ID */
		PRINT_ID_STATE /**< Print ID and state */
	} PrintState;

	SkelGuiSkeletonDrawer3D(fawkes::openni::UserMap &users, fawkes::openni::HandMap &hands);

	void draw();

	void toggle_print_state();
	void set_print_state(PrintState state);

private:
	void print_string(void *font, char *str);
	void draw_limb(float *p1, float conf1, float *p2, float conf2);
	void draw_user(fawkes::openni::UserInfo &user);
	void draw_circle(unsigned int id, float *p, float radius);

private:
	fawkes::openni::UserMap &users_;
	fawkes::openni::HandMap &hands_;

	PrintState print_state_;
};

#endif
