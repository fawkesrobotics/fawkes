
/***************************************************************************
 *  motion_utils.h - Motion utility functions
 *
 *  Created: Wed Aug 17 21:51:51 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_NAO_MOTION_UTILS_H_
#define __PLUGINS_NAO_MOTION_UTILS_H_

#include <alcore/alptr.h>
#include <alproxies/almotionproxy.h>

#include <vector>
#include <string>

namespace motion {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

void
fix_angles(AL::ALPtr<AL::ALMotionProxy> &almotion);

void
move_joints(AL::ALPtr<AL::ALMotionProxy> &almotion,
            float head_yaw, float head_pitch,
            float l_shoulder_pitch, float l_shoulder_roll,
            float l_elbow_yaw, float l_elbow_roll,
            float l_wrist_yaw, float l_hand,
            float l_hip_yaw_pitch, float l_hip_roll, float l_hip_pitch,
            float l_knee_pitch, float l_ankle_pitch, float l_ankle_roll,
            float r_shoulder_pitch, float r_shoulder_roll,
            float r_elbow_yaw, float r_elbow_roll,
            float r_wrist_yaw, float r_hand,
            float r_hip_yaw_pitch, float r_hip_roll, float r_hip_pitch,
            float r_knee_pitch, float r_ankle_pitch, float r_ankle_roll,
            float speed);

int
timed_move_joints(AL::ALPtr<AL::ALMotionProxy> &almotion,
                  float head_yaw, float head_pitch,
                  float l_shoulder_pitch, float l_shoulder_roll,
                  float l_elbow_yaw, float l_elbow_roll,
                  float l_wrist_yaw, float l_hand,
                  float l_hip_yaw_pitch, float l_hip_roll, float l_hip_pitch,
                  float l_knee_pitch, float l_ankle_pitch, float l_ankle_roll,
                  float r_shoulder_pitch, float r_shoulder_roll,
                  float r_elbow_yaw, float r_elbow_roll,
                  float r_wrist_yaw, float r_hand,
                  float r_hip_yaw_pitch, float r_hip_roll, float r_hip_pitch,
                  float r_knee_pitch, float r_ankle_pitch, float r_ankle_roll,
                  float time_sec);

} // end of namespace motion

#endif
