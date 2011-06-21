
/***************************************************************************
 *  motion_thread.cpp - Provide NaoQi motion to Fawkes
 *
 *  Created: Thu Jun 09 12:58:14 2011
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

#include "motion_thread.h"
#include "motion_kick_task.h"
#include "motion_standup_task.h"

#include <alcore/alerror.h>
#include <alproxies/allauncherproxy.h>
#include <alproxies/almotionproxy.h>
#include <althread/althreadpool.h>

#include <interfaces/HumanoidMotionInterface.h>
#include <interfaces/NaoSensorInterface.h>

using namespace fawkes;

/** @class NaoQiMotionThread "motion_thread.h"
 * Thread to provide NaoQi motions to Fawkes.
 * This thread holds an ALMotion proxy and provides its capabilities via
 * the blackboard to other Fawkes threads.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
NaoQiMotionThread::NaoQiMotionThread()
  : Thread("NaoQiMotionThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}


/** Destructor. */
NaoQiMotionThread::~NaoQiMotionThread()
{
}


void
NaoQiMotionThread::init()
{
  __motion_task_id = -1;

  // Is ALMotion available?
  try {
    AL::ALPtr<AL::ALLauncherProxy> launcher(new AL::ALLauncherProxy(naoqi_broker));
    bool is_almotion_available = launcher->isModulePresent("ALMotion");

    if (! is_almotion_available) {
      throw Exception("NaoQi ALMotion is not available");
    }
  } catch (AL::ALError& e) {
    throw Exception("Checking ALMotion aliveness failed: %s",
		    e.toString().c_str());
  }

  __almotion = naoqi_broker->getMotionProxy();
  __thread_pool = naoqi_broker->getThreadPool();

  __hummot_if =
    blackboard->open_for_writing<HumanoidMotionInterface>("NaoQi Motion");
  __sensor_if =
    blackboard->open_for_reading<NaoSensorInterface>("Nao Sensors");
}


void
NaoQiMotionThread::finalize()
{
  stop_motion();

  blackboard->close(__hummot_if);
  blackboard->close(__sensor_if);
  __hummot_if = NULL;
  __sensor_if = NULL;

  __almotion.reset();
}


/** Stop any currently running motion.
 * Walk tasks are stopped gracefully, i.e. by setting the velocity to zero and
 * allowing the robot to come to a safe stopping position. Other motion
 * tasks are simply killed.
 */
void
NaoQiMotionThread::stop_motion()
{
  if (__almotion->walkIsActive()) {
    __almotion->setWalkTargetVelocity(0., 0., 0., 0.);
  } else if (__motion_task_id != -1) {
    if (__almotion->isRunning(__motion_task_id)) {
      __almotion->killTask(__motion_task_id);
    }
    __motion_task_id = -1;
  } else if (__motion_task) {
    if (__motion_task) {
      __motion_task->exitTask();
      __motion_task.reset();
    }
  }
}


/** Fix ALMotion's belief of body angles.
 * If body angles have been set via the DCM, ALMotions model is out of
 * date which can cause a quick snapping back to the last posture known
 * to ALMotion causing very fast and potentially dangerous movements.
 *
 * Seems not to work as expected atm.
 */
void
NaoQiMotionThread::fix_angles()
{

  __almotion->setAngles("Body", __almotion->getAngles("Body", true), 1.);
  //__almotion->setStiffnesses("Body", 0.0);
  //__almotion->setStiffnesses("Body", 1.0);
}


void
NaoQiMotionThread::loop()
{
  process_messages();

  bool walking = __almotion->walkIsActive();
  bool tasking = __motion_task_id != -1 && __almotion->isRunning(__motion_task_id);
  bool custom_task  = false;

  if (__motion_task) {
    if (__motion_task->getState() == AL::ALTask::RUNNING) {
      custom_task = true;
    } else if (__motion_task->getState() == AL::ALTask::ENDED) {
      __motion_task.reset();
    }
  }

  __hummot_if->set_moving(walking || tasking || custom_task);
  AL::ALValue varms_enabled = __almotion->getWalkArmsEnable();
  bool arms_enabled = varms_enabled[0] || varms_enabled[1];
  __hummot_if->set_arms_enabled(arms_enabled);
  __hummot_if->write();
}


/** Process incoming BB messages. */
void
NaoQiMotionThread::process_messages()
{
  // process bb messages
  if ( ! __hummot_if->msgq_empty() ) {
    if (__hummot_if->msgq_first_is<HumanoidMotionInterface::StopMessage>())
    {
      logger->log_debug(name(), "Stopping motion");
      stop_motion();

    }
    else if (HumanoidMotionInterface::WalkStraightMessage *msg =
	       __hummot_if->msgq_first_safe(msg))
    {

    }
    else if (HumanoidMotionInterface::WalkVelocityMessage *msg =
	     __hummot_if->msgq_first_safe(msg))
    {
      if ( (msg->speed() < 0.) || (msg->speed() > 1.)) {
	logger->log_warn(name(), "Walk velocity speed %f out of range [0.0..1.0],",
			 "ignoring command", msg->speed());
      } else {
	try {
	  __almotion->setWalkTargetVelocity(msg->x(), msg->y(), msg->theta(),
					    msg->speed());
	} catch (AL::ALError &e) {
	  logger->log_warn(name(), "WalkVelocity command failed: %s", e.what());
	}
      }
      __hummot_if->set_msgid(msg->id());
    }

    else if (HumanoidMotionInterface::GetUpMessage *msg =
	     __hummot_if->msgq_first_safe(msg))
    {
      goto_body_angles(/* head */ 0., 0.,
		       /* l shoulder */ 2.1, 0.35,
		       /* l elbow */ -1.40, -1.40, 
		       /* l wrist/hand */ 0., 0.,
		       /* l hip */ 0., 0., -0.52,
		       /* l knee */ 1.05,
		       /* l ankle */ -0.52, 0.,
		       /* r shoulder */ 2.1, -0.35, 
		       /* r elbow */ 1.40, 1.40,
		       /* r wrist/hand */ 0., 0.,
		       /* r hip */ 0., 0., -0.52,
		       /* r knee */ 1.05,
		       /* r ankle */ -0.52, 0.,
		       /* time */ 3.0);

      __hummot_if->set_msgid(msg->id());
    }
    else if (HumanoidMotionInterface::ParkMessage *msg =
	     __hummot_if->msgq_first_safe(msg))
    {
      goto_body_angles(/* head */ 0., 0.,
		       /* l shoulder */ 1.58, 0.15,
		       /* l elbow */ -1.20, -1.1,
		       /* l wrist/hand */ 0., 0.,
		       /* l hip */ -0.08, 0., -0.85,
		       /* l knee */ 2.2,
		       /* l ankle */ -1.23, 0.,
		       /* r shoulder */ 1.55, -0.15,
		       /* r elbow */ 1.2, 1.1,
		       /* r wrist/hand */ 0., 0.,
		       /* r hip */ -0.08, 0., -0.85,
		       /* r knee */ 2.2,
		       /* r ankle */ -1.23, 0.,
		       /* time */ 3.0);

      __hummot_if->set_msgid(msg->id());
    }

    else if (HumanoidMotionInterface::KickMessage *msg =
	     __hummot_if->msgq_first_safe(msg))
    {
      if (__motion_task) {
	__motion_task->exitTask();
      }
      __motion_task.reset(new NaoQiMotionKickTask(__almotion, msg->leg()));
      __thread_pool->enqueue(__motion_task);

      __hummot_if->set_msgid(msg->id());
    }

    else if (HumanoidMotionInterface::StandupMessage *msg =
	     __hummot_if->msgq_first_safe(msg))
    {
      if (__motion_task) {
	__motion_task->exitTask();
      }

      __sensor_if->read();
      __motion_task.reset(new NaoQiMotionStandupTask(__almotion, msg->from_pos(),
						     __sensor_if->accel_x(),
						     __sensor_if->accel_y(),
						     __sensor_if->accel_z()));
      __thread_pool->enqueue(__motion_task);

      __hummot_if->set_msgid(msg->id());
    }

    __hummot_if->msgq_pop();
  }
}


void
NaoQiMotionThread::goto_body_angles(float head_yaw, float head_pitch,
				    float l_shoulder_pitch, float l_shoulder_roll,
				    float l_elbow_yaw, float l_elbow_roll,
				    float l_wrist_yaw, float l_hand,
				    float l_hip_yaw_pitch, float l_hip_roll,
				    float l_hip_pitch, float l_knee_pitch,
				    float l_ankle_pitch, float l_ankle_roll,
				    float r_shoulder_pitch, float r_shoulder_roll,
				    float r_elbow_yaw, float r_elbow_roll,
				    float r_wrist_yaw, float r_hand,
				    float r_hip_yaw_pitch, float r_hip_roll,
				    float r_hip_pitch, float r_knee_pitch,
				    float r_ankle_pitch, float r_ankle_roll,
				    float time_sec)
{
  int num_joints = __almotion->getJointNames("Body").size();

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

  std::vector<std::string> joint_names = __almotion->getJointNames("Body");
  __almotion->killTasksUsingResources(joint_names);

  fix_angles();

  __motion_task_id =
    __almotion->post.angleInterpolation("Body", angles, time_sec, true);

  /*
  for (unsigned int i = 0; i < 160; ++i) {
    std::vector<float> angles2 = __almotion->getAngles("Body", false);
    std::vector<float> angles3 = __almotion->getAngles("Body", true);
    printf("3-- LShoulderPitch: cmd %f  sensed %f\n", angles2[2], angles3[2]);
    usleep(20000);
  }
  */
}
