
/***************************************************************************
 *  motion_utils.cpp - Motion utility functions
 *
 *  Created: Wed Aug 17 21:54:59 2011
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

#include "motion_utils.h"

namespace motion {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Fix ALMotion's belief of body angles.
 * If body angles have been set via the DCM, ALMotions model is out of
 * date which can cause a quick snapping back to the last posture known
 * to ALMotion causing very fast and potentially dangerous movements.
 *
 * Seems not to work as expected atm.
 */
void
fix_angles(AL::ALPtr<AL::ALMotionProxy> &almotion)
{

  almotion->setAngles("Body", almotion->getAngles("Body", true), 1.);
  //almotion->setStiffnesses("Body", 0.0);
  //almotion->setStiffnesses("Body", 1.0);
}

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
            float speed)
{
  int num_joints = almotion->getJointNames("Body").size();

  std::vector<float> angles;
  angles.push_back(head_yaw);
  angles.push_back(head_pitch);

  angles.push_back(l_shoulder_pitch);
  angles.push_back(l_shoulder_roll);
  angles.push_back(l_elbow_yaw);
  angles.push_back(l_elbow_roll);
  if (num_joints == 26) { //academic version
    angles.push_back(l_wrist_yaw);
    angles.push_back(l_hand);
  }

  angles.push_back(l_hip_yaw_pitch);
  angles.push_back(l_hip_roll);
  angles.push_back(l_hip_pitch);
  angles.push_back(l_knee_pitch);
  angles.push_back(l_ankle_pitch);
  angles.push_back(l_ankle_roll);

  angles.push_back(r_hip_yaw_pitch);
  angles.push_back(r_hip_roll);
  angles.push_back(r_hip_pitch);
  angles.push_back(r_knee_pitch);
  angles.push_back(r_ankle_pitch);
  angles.push_back(r_ankle_roll);

  angles.push_back(r_shoulder_pitch);
  angles.push_back(r_shoulder_roll);
  angles.push_back(r_elbow_yaw);
  angles.push_back(r_elbow_roll);
  if (num_joints == 26) {
    angles.push_back(r_wrist_yaw);
    angles.push_back(r_hand);
  }

  std::vector<std::string> joint_names = almotion->getJointNames("Body");
  almotion->killTasksUsingResources(joint_names);

  fix_angles(almotion);

  almotion->setAngles(joint_names, angles, speed);
}


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
                  float time_sec)
{
  int num_joints = almotion->getJointNames("Body").size();

  std::vector<float> angles;
  angles.push_back(head_yaw);
  angles.push_back(head_pitch);

  angles.push_back(l_shoulder_pitch);
  angles.push_back(l_shoulder_roll);
  angles.push_back(l_elbow_yaw);
  angles.push_back(l_elbow_roll);
  if (num_joints == 26) { //academic version
    angles.push_back(l_wrist_yaw);
    angles.push_back(l_hand);
  }

  angles.push_back(l_hip_yaw_pitch);
  angles.push_back(l_hip_roll);
  angles.push_back(l_hip_pitch);
  angles.push_back(l_knee_pitch);
  angles.push_back(l_ankle_pitch);
  angles.push_back(l_ankle_roll);

  angles.push_back(r_hip_yaw_pitch);
  angles.push_back(r_hip_roll);
  angles.push_back(r_hip_pitch);
  angles.push_back(r_knee_pitch);
  angles.push_back(r_ankle_pitch);
  angles.push_back(r_ankle_roll);

  angles.push_back(r_shoulder_pitch);
  angles.push_back(r_shoulder_roll);
  angles.push_back(r_elbow_yaw);
  angles.push_back(r_elbow_roll);
  if (num_joints == 26) {
    angles.push_back(r_wrist_yaw);
    angles.push_back(r_hand);
  }

  std::vector<std::string> joint_names = almotion->getJointNames("Body");
  almotion->killTasksUsingResources(joint_names);

  fix_angles(almotion);

  return almotion->post.angleInterpolation("Body", angles, time_sec, true);
}


} // end of namespace motion
