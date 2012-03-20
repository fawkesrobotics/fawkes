
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
#include "motion_utils.h"

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


  AL::ALValue names  = AL::ALValue::array("HeadYaw", "HeadPitch");
  std::vector<float> head_angles = __almotion->getAngles(names, false);
  __almotion->setAngles(names, head_angles, 1.0);
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
  bool stop = false;
  HumanoidMotionInterface::WalkVelocityMessage*  msg_walk_velocity = NULL;
  HumanoidMotionInterface::MoveHeadMessage*      msg_move_head = NULL;
  Message* msg_action = NULL;

  // process bb messages
  while ( ! __hummot_if->msgq_empty() ) {
    if (__hummot_if->msgq_first_is<HumanoidMotionInterface::StopMessage>())
    {
      stop = true;
    }
    else if (HumanoidMotionInterface::WalkStraightMessage *msg =
	       __hummot_if->msgq_first_safe(msg))
    {

    }
    else if (HumanoidMotionInterface::WalkVelocityMessage *msg =
	     __hummot_if->msgq_first_safe(msg))
    {
      if (msg_walk_velocity)  msg_walk_velocity->unref();
      msg_walk_velocity = msg;
      msg_walk_velocity->ref();
      if (msg_action)  msg_action->unref();
      msg_action = NULL;
      stop = false;
    }

    else if (HumanoidMotionInterface::MoveHeadMessage *msg =
	     __hummot_if->msgq_first_safe(msg))
    {
      if (msg_move_head)  msg_move_head->unref();
      msg_move_head = msg;
      msg_move_head->ref();
      if (msg_action)  msg_action->unref();
      msg_action = NULL;
      stop = false;
    }

    else if (HumanoidMotionInterface::GetUpMessage *msg =
	     __hummot_if->msgq_first_safe(msg))
    {
      if (msg_action)  msg_action->unref();
      msg_action = msg;
      msg_action->ref();
      stop = false;
    }
    else if (HumanoidMotionInterface::ParkMessage *msg =
	     __hummot_if->msgq_first_safe(msg))
    {
      if (msg_action)  msg_action->unref();
      msg_action = msg;
      msg_action->ref();
      stop = false;
    }

    else if (HumanoidMotionInterface::KickMessage *msg =
	     __hummot_if->msgq_first_safe(msg))
    {
      if (msg_action)  msg_action->unref();
      msg_action = msg;
      msg_action->ref();
      stop = false;
    }

    else if (HumanoidMotionInterface::StandupMessage *msg =
	     __hummot_if->msgq_first_safe(msg))
    {
      if (msg_action)  msg_action->unref();
      msg_action = msg;
      msg_action->ref();
      stop = false;
    }

    __hummot_if->msgq_pop();
  }

  // process last message
  if (stop) {
    logger->log_debug(name(), "Stopping motion");
    stop_motion();
  }
  else if (msg_action) {
    if (msg_action->is_of_type<HumanoidMotionInterface::GetUpMessage>()) {
      motion::timed_move_joints(__almotion,
                                /* head */ 0., 0.,
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

      __hummot_if->set_msgid(msg_action->id());
    }
    else if (msg_action->is_of_type<HumanoidMotionInterface::ParkMessage>()) {
      motion::timed_move_joints(__almotion,
                                /* head */ 0., 0.,
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

      __hummot_if->set_msgid(msg_action->id());
    }
    else if (msg_action->is_of_type<HumanoidMotionInterface::StandupMessage>()) {
      if (__motion_task) {
	__motion_task->exitTask();
      }

      HumanoidMotionInterface::StandupMessage* msg = dynamic_cast<HumanoidMotionInterface::StandupMessage*>(msg_action);
      __sensor_if->read();
      __motion_task.reset(new NaoQiMotionStandupTask(__almotion, msg->from_pos(),
						     __sensor_if->accel_x(),
						     __sensor_if->accel_y(),
						     __sensor_if->accel_z()));
      __thread_pool->enqueue(__motion_task);

      __hummot_if->set_msgid(msg->id());
    }
    else if (msg_action->is_of_type<HumanoidMotionInterface::KickMessage>()) {
      HumanoidMotionInterface::KickMessage* msg = dynamic_cast<HumanoidMotionInterface::KickMessage*>(msg_action);
      if (__motion_task) {
	__motion_task->exitTask();
      }
      __motion_task.reset(new NaoQiMotionKickTask(__almotion, msg->leg()));
      __thread_pool->enqueue(__motion_task);

      __hummot_if->set_msgid(msg->id());
    }

    msg_action->unref();
  }
  else {
    if (msg_walk_velocity) {
      if ( (msg_walk_velocity->speed() < 0.) || (msg_walk_velocity->speed() > 1.)) {
	logger->log_warn(name(), "Walk velocity speed %f out of range [0.0..1.0],",
			 "ignoring command", msg_walk_velocity->speed());
      } else {
	try {
	  __almotion->setWalkTargetVelocity(msg_walk_velocity->x(), msg_walk_velocity->y(), msg_walk_velocity->theta(),
					    msg_walk_velocity->speed());
	} catch (AL::ALError &e) {
	  logger->log_warn(name(), "WalkVelocity command failed: %s", e.what());
	}
      }
      __hummot_if->set_msgid(msg_walk_velocity->id());
      msg_walk_velocity->unref();
    }

    if (msg_move_head) {
      AL::ALValue names  = AL::ALValue::array("HeadYaw", "HeadPitch");
      AL::ALValue angles = AL::ALValue::array(msg_move_head->yaw(), msg_move_head->pitch());

      __almotion->setAngles(names, angles, msg_move_head->speed());
      msg_move_head->unref();
    }
  }



}
