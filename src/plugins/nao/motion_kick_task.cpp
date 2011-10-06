
/***************************************************************************
 *  motion_kick_task.cpp - Make the robot kick asses... ehm soccer balls
 *
 *  Created: Fri Jan 23 18:36:01 2009
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *  		    2010  Patrick Podbregar [www.podbregar.com]
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

#include "motion_kick_task.h"

#include <core/exceptions/system.h>
#include <utils/math/angle.h>

#include <cstring>
#include <cstdlib>
#include <string>
#include <unistd.h>

using namespace AL;
using namespace fawkes;

/** @class NaoQiMotionKickTask "motion_kick_task.h"
 * NaoQi kick task.
 * This task can be used to make the robot kick in a non-blocking way. It will
 * use (blocking) ALMotion calls to execute the move. Note that ALMotion should
 * not be used otherwise while kicking.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param almotion ALMotion proxy
 * @param leg leg to kick with
 */
NaoQiMotionKickTask::NaoQiMotionKickTask(AL::ALPtr<AL::ALMotionProxy> almotion,
					 fawkes::HumanoidMotionInterface::LegEnum leg)
{
  __quit      = false;
  __almotion  = almotion;
  __leg       = leg;

  // ALTask variable to cause auto-destruct when done
  fAutoDelete = true;
}


/** Destructor. */
NaoQiMotionKickTask::~NaoQiMotionKickTask()
{
}


/*
static void
print_angles(std::vector<float> angles)
{
  for (std::vector<float>::iterator i = angles.begin(); i != angles.end(); ++i) {
    printf("%f, ", *i);
  }
  printf("\n");
  for (std::vector<float>::iterator i = angles.begin(); i != angles.end(); ++i) {
    printf("%f, ", rad2deg(*i));
  }
  printf("\n\n\n");
}
*/

void
NaoQiMotionKickTask::goto_start_pos(AL::ALValue speed, bool concurrent)
{
  float shoulder_pitch = 120;
  float shoulder_roll = 15;
  float elbow_yaw = -90;
  float elbow_roll = -80;
  float hip_yaw_pitch = 0;
  float hip_roll = 0;
  float hip_pitch = -25;
  float knee_pitch = 40;
  float ankle_pitch = -20;
  float ankle_roll = 0;

  ALValue target_angles;
  target_angles.arraySetSize(20);

  // left arm
  target_angles[0] = deg2rad(shoulder_pitch);
  target_angles[1] = deg2rad(shoulder_roll);
  target_angles[2] = deg2rad(elbow_yaw);
  target_angles[3] = deg2rad(elbow_roll);
  // right arm
  target_angles[4] = deg2rad(shoulder_pitch);
  target_angles[5] = -deg2rad(shoulder_roll);
  target_angles[6] = -deg2rad(elbow_yaw);
  target_angles[7] = -deg2rad(elbow_roll);
  // left leg
  target_angles[8] = deg2rad(hip_yaw_pitch);
  target_angles[9] = deg2rad(hip_roll);
  target_angles[10] = deg2rad(hip_pitch);
  target_angles[11] = deg2rad(knee_pitch);
  target_angles[12] = deg2rad(ankle_pitch);
  target_angles[13] = deg2rad(ankle_roll);
  // right leg
  target_angles[14] = deg2rad(hip_yaw_pitch);
  target_angles[15] = deg2rad(hip_roll);
  target_angles[16] = deg2rad(hip_pitch);
  target_angles[17] = deg2rad(knee_pitch);
  target_angles[18] = deg2rad(ankle_pitch);
  target_angles[19] = -deg2rad(ankle_roll);

  ALValue names = ALValue::array("LArm", "RArm", "LLeg", "RLeg");

  if (concurrent) {
    __almotion->post.angleInterpolationWithSpeed(names, target_angles, speed);
  } else {
    __almotion->angleInterpolationWithSpeed(names, target_angles, speed);
  }
}


/** Stop the current kick task.
 * Stops the current motion and posts a goto for the start position.
 * This is not stable from all configurations but seems to suffices
 * most of the time.
 */
void
NaoQiMotionKickTask::exitTask()
{
  __quit = true;
  std::vector<std::string> joint_names = __almotion->getJointNames("Body");
  __almotion->killTasksUsingResources(joint_names);
  goto_start_pos(0.2, true);
}

/** Run the kick. */
void
NaoQiMotionKickTask::run()
{
  const char *shoot_hip_roll_name = NULL;
  const char *support_hip_roll_name = NULL;
  const char *shoot_hip_pitch_name = NULL;
  const char *support_hip_pitch_name = NULL;
  const char *shoot_knee_pitch_name = NULL;
  const char *shoot_ankle_pitch_name = NULL;
  const char *shoot_ankle_roll_name = NULL;
  const char *support_ankle_roll_name = NULL;

  float shoot_hip_roll = 0;
  float support_hip_roll = 0;
  float shoot_hip_pitch = 0;
  float support_hip_pitch = 0;
  float shoot_knee_pitch = 0;
  float shoot_ankle_pitch = 0;
  float shoot_ankle_roll = 0;
  float support_ankle_roll = 0;

  float BALANCE_HIP_ROLL = 0;
  float BALANCE_ANKLE_ROLL = 0;
  float STRIKE_OUT_HIP_ROLL = 0;

  if ( __leg == fawkes::HumanoidMotionInterface::LEG_LEFT ) {
    shoot_hip_roll_name = "LHipRoll";
    support_hip_roll_name = "RHipRoll";
    shoot_hip_pitch_name = "LHipPitch";
    support_hip_pitch_name = "RHipPitch";
    shoot_knee_pitch_name = "LKneePitch";
    shoot_ankle_pitch_name = "LAnklePitch";
    shoot_ankle_roll_name = "LAnkleRoll";
    support_ankle_roll_name = "RAnkleRoll";

    BALANCE_HIP_ROLL = 20;
    BALANCE_ANKLE_ROLL = -25;
    STRIKE_OUT_HIP_ROLL = 30;
  } else if (__leg == fawkes::HumanoidMotionInterface::LEG_RIGHT ) {
    shoot_hip_roll_name = "RHipRoll";
    support_hip_roll_name = "LHipRoll";
    shoot_hip_pitch_name = "RHipPitch";
    support_hip_pitch_name = "LHipPitch";
    shoot_knee_pitch_name = "RKneePitch";
    shoot_ankle_pitch_name = "RAnklePitch";
    shoot_ankle_roll_name = "RAnkleRoll";
    support_ankle_roll_name = "LAnkleRoll";

    BALANCE_HIP_ROLL = -20;
    BALANCE_ANKLE_ROLL = 25;
    STRIKE_OUT_HIP_ROLL = -30;
  }

  if (__quit)  return;
  goto_start_pos(0.2);

  ALValue names;
  ALValue target_angles;
  float speed = 0;

  // Balance on supporting leg
  names.arraySetSize(4);
  target_angles.arraySetSize(4);

  support_hip_roll = BALANCE_HIP_ROLL;
  shoot_hip_roll = BALANCE_HIP_ROLL;
  shoot_ankle_roll = BALANCE_ANKLE_ROLL;
  support_ankle_roll = BALANCE_ANKLE_ROLL;

  names = ALValue::array(support_hip_roll_name, shoot_hip_roll_name,
      support_ankle_roll_name, shoot_ankle_roll_name);
  target_angles = ALValue::array(deg2rad(support_hip_roll), deg2rad(shoot_hip_roll),
      deg2rad(support_ankle_roll), deg2rad(shoot_ankle_roll));
  speed = 0.15;

  //if (__quit)  return;
  __almotion->angleInterpolationWithSpeed(names, target_angles, speed);

  names.clear();
  target_angles.clear();

  // Raise shooting leg
  names.arraySetSize(3);
  target_angles.arraySetSize(3);

  shoot_hip_roll = STRIKE_OUT_HIP_ROLL;
  shoot_knee_pitch = 90;
  shoot_ankle_pitch = -50;

  names = ALValue::array(shoot_hip_roll_name, shoot_knee_pitch_name, shoot_ankle_pitch_name);
  target_angles = ALValue::array(deg2rad(shoot_hip_roll), deg2rad(shoot_knee_pitch),
      deg2rad(shoot_ankle_pitch));
  speed = 0.2;

  if (__quit)  return;
  __almotion->angleInterpolationWithSpeed(names, target_angles, speed);

  names.clear();
  target_angles.clear();

  // Strike out
  names.arraySetSize(2);
  target_angles.arraySetSize(2);

  shoot_hip_pitch = 20;
  support_hip_pitch = -50;

  names = ALValue::array(shoot_hip_pitch_name, support_hip_pitch_name);
  target_angles = ALValue::array(deg2rad(shoot_hip_pitch), deg2rad(support_hip_pitch));
  speed = 0.1;

  if (__quit)  return;
  __almotion->angleInterpolationWithSpeed(names, target_angles, speed);

  names.clear();
  target_angles.clear();

  // Shoot
  names.arraySetSize(4);
  target_angles.arraySetSize(4);

  shoot_hip_pitch = -80;
  support_hip_pitch = -40;
  shoot_knee_pitch = 50;
  shoot_ankle_pitch = -30;

  names = ALValue::array(shoot_hip_pitch_name, support_hip_pitch_name,
      shoot_knee_pitch_name, shoot_ankle_pitch_name);
  target_angles = ALValue::array(deg2rad(shoot_hip_pitch), deg2rad(support_hip_pitch),
      deg2rad(shoot_knee_pitch), deg2rad(shoot_ankle_pitch));
  speed = 1.0;
  if (__quit)  return;
  __almotion->angleInterpolationWithSpeed(names, target_angles, speed);

  names.clear();
  target_angles.clear();

  // Move to upright position
  names.arraySetSize(2);
  target_angles.arraySetSize(2);

  shoot_hip_pitch = -25;
  support_hip_pitch = -25;

  names = ALValue::array(shoot_hip_pitch_name, support_hip_pitch_name);
  target_angles = ALValue::array(deg2rad(shoot_hip_pitch), deg2rad(support_hip_pitch));
  speed = 0.1;

  if (__quit)  return;
  __almotion->angleInterpolationWithSpeed(names, target_angles, speed);

  //names.clear();
  //target_angles.clear();

  if (__quit)  return;
  goto_start_pos(0.1);
}
