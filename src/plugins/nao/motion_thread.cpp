
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
	motion_task_id_ = -1;

	// Is ALMotion available?
	try {
		AL::ALPtr<AL::ALLauncherProxy> launcher(new AL::ALLauncherProxy(naoqi_broker));
		bool                           is_almotion_available = launcher->isModulePresent("ALMotion");

		if (!is_almotion_available) {
			throw Exception("NaoQi ALMotion is not available");
		}
	} catch (AL::ALError &e) {
		throw Exception("Checking ALMotion aliveness failed: %s", e.toString().c_str());
	}

	almotion_    = naoqi_broker->getMotionProxy();
	thread_pool_ = naoqi_broker->getThreadPool();

	hummot_if_ = blackboard->open_for_writing<HumanoidMotionInterface>("NaoQi Motion");
	sensor_if_ = blackboard->open_for_reading<NaoSensorInterface>("Nao Sensors");
}

void
NaoQiMotionThread::finalize()
{
	stop_motion();

	blackboard->close(hummot_if_);
	blackboard->close(sensor_if_);
	hummot_if_ = NULL;
	sensor_if_ = NULL;

	almotion_.reset();
}

/** Stop any currently running motion.
 * Walk tasks are stopped gracefully, i.e. by setting the velocity to zero and
 * allowing the robot to come to a safe stopping position. Other motion
 * tasks are simply killed.
 */
void
NaoQiMotionThread::stop_motion()
{
	if (almotion_->walkIsActive()) {
		almotion_->setWalkTargetVelocity(0., 0., 0., 0.);
	} else if (motion_task_id_ != -1) {
		if (almotion_->isRunning(motion_task_id_)) {
			almotion_->killTask(motion_task_id_);
		}
		motion_task_id_ = -1;
	} else if (motion_task_) {
		if (motion_task_) {
			motion_task_->exitTask();
			motion_task_.reset();
		}
	}

	AL::ALValue        names       = AL::ALValue::array("HeadYaw", "HeadPitch");
	std::vector<float> head_angles = almotion_->getAngles(names, false);
	almotion_->setAngles(names, head_angles, 1.0);
}

void
NaoQiMotionThread::loop()
{
	process_messages();

	bool walking     = almotion_->walkIsActive();
	bool tasking     = motion_task_id_ != -1 && almotion_->isRunning(motion_task_id_);
	bool custom_task = false;

	if (motion_task_) {
		if (motion_task_->getState() == AL::ALTask::RUNNING) {
			custom_task = true;
		} else if (motion_task_->getState() == AL::ALTask::ENDED) {
			motion_task_.reset();
		}
	}

	hummot_if_->set_moving(walking || tasking || custom_task);
	AL::ALValue varms_enabled = almotion_->getWalkArmsEnable();
	bool        arms_enabled  = varms_enabled[0] || varms_enabled[1];
	hummot_if_->set_arms_enabled(arms_enabled);
	hummot_if_->write();
}

/** Process incoming BB messages. */
void
NaoQiMotionThread::process_messages()
{
	bool                                          stop              = false;
	HumanoidMotionInterface::WalkVelocityMessage *msg_walk_velocity = NULL;
	HumanoidMotionInterface::MoveHeadMessage *    msg_move_head     = NULL;
	Message *                                     msg_action        = NULL;

	// process bb messages
	while (!hummot_if_->msgq_empty()) {
		if (hummot_if_->msgq_first_is<HumanoidMotionInterface::StopMessage>()) {
			stop = true;
		} else if (HumanoidMotionInterface::WalkStraightMessage *msg =
		             hummot_if_->msgq_first_safe(msg)) {
		} else if (HumanoidMotionInterface::WalkVelocityMessage *msg =
		             hummot_if_->msgq_first_safe(msg)) {
			if (msg_walk_velocity)
				msg_walk_velocity->unref();
			msg_walk_velocity = msg;
			msg_walk_velocity->ref();
			if (msg_action)
				msg_action->unref();
			msg_action = NULL;
			stop       = false;
		}

		else if (HumanoidMotionInterface::MoveHeadMessage *msg = hummot_if_->msgq_first_safe(msg)) {
			if (msg_move_head)
				msg_move_head->unref();
			msg_move_head = msg;
			msg_move_head->ref();
			if (msg_action)
				msg_action->unref();
			msg_action = NULL;
			stop       = false;
		}

		else if (HumanoidMotionInterface::GetUpMessage *msg = hummot_if_->msgq_first_safe(msg)) {
			if (msg_action)
				msg_action->unref();
			msg_action = msg;
			msg_action->ref();
			stop = false;
		} else if (HumanoidMotionInterface::ParkMessage *msg = hummot_if_->msgq_first_safe(msg)) {
			if (msg_action)
				msg_action->unref();
			msg_action = msg;
			msg_action->ref();
			stop = false;
		}

		else if (HumanoidMotionInterface::KickMessage *msg = hummot_if_->msgq_first_safe(msg)) {
			if (msg_action)
				msg_action->unref();
			msg_action = msg;
			msg_action->ref();
			stop = false;
		}

		else if (HumanoidMotionInterface::StandupMessage *msg = hummot_if_->msgq_first_safe(msg)) {
			if (msg_action)
				msg_action->unref();
			msg_action = msg;
			msg_action->ref();
			stop = false;
		}

		hummot_if_->msgq_pop();
	}

	// process last message
	if (stop) {
		logger->log_debug(name(), "Stopping motion");
		stop_motion();
	} else if (msg_action) {
		if (msg_action->is_of_type<HumanoidMotionInterface::GetUpMessage>()) {
			motion::timed_move_joints(almotion_,
			                          /* head */ 0.,
			                          0.,
			                          /* l shoulder */ 2.1,
			                          0.35,
			                          /* l elbow */ -1.40,
			                          -1.40,
			                          /* l wrist/hand */ 0.,
			                          0.,
			                          /* l hip */ 0.,
			                          0.,
			                          -0.52,
			                          /* l knee */ 1.05,
			                          /* l ankle */ -0.52,
			                          0.,
			                          /* r shoulder */ 2.1,
			                          -0.35,
			                          /* r elbow */ 1.40,
			                          1.40,
			                          /* r wrist/hand */ 0.,
			                          0.,
			                          /* r hip */ 0.,
			                          0.,
			                          -0.52,
			                          /* r knee */ 1.05,
			                          /* r ankle */ -0.52,
			                          0.,
			                          /* time */ 3.0);

			hummot_if_->set_msgid(msg_action->id());
		} else if (msg_action->is_of_type<HumanoidMotionInterface::ParkMessage>()) {
			motion::timed_move_joints(almotion_,
			                          /* head */ 0.,
			                          0.,
			                          /* l shoulder */ 1.58,
			                          0.15,
			                          /* l elbow */ -1.20,
			                          -1.1,
			                          /* l wrist/hand */ 0.,
			                          0.,
			                          /* l hip */ -0.08,
			                          0.,
			                          -0.85,
			                          /* l knee */ 2.2,
			                          /* l ankle */ -1.23,
			                          0.,
			                          /* r shoulder */ 1.55,
			                          -0.15,
			                          /* r elbow */ 1.2,
			                          1.1,
			                          /* r wrist/hand */ 0.,
			                          0.,
			                          /* r hip */ -0.08,
			                          0.,
			                          -0.85,
			                          /* r knee */ 2.2,
			                          /* r ankle */ -1.23,
			                          0.,
			                          /* time */ 3.0);

			hummot_if_->set_msgid(msg_action->id());
		} else if (msg_action->is_of_type<HumanoidMotionInterface::StandupMessage>()) {
			if (motion_task_) {
				motion_task_->exitTask();
			}

			HumanoidMotionInterface::StandupMessage *msg =
			  dynamic_cast<HumanoidMotionInterface::StandupMessage *>(msg_action);
			sensor_if_->read();
			motion_task_.reset(new NaoQiMotionStandupTask(almotion_,
			                                              msg->from_pos(),
			                                              sensor_if_->accel_x(),
			                                              sensor_if_->accel_y(),
			                                              sensor_if_->accel_z()));
			thread_pool_->enqueue(motion_task_);

			hummot_if_->set_msgid(msg->id());
		} else if (msg_action->is_of_type<HumanoidMotionInterface::KickMessage>()) {
			HumanoidMotionInterface::KickMessage *msg =
			  dynamic_cast<HumanoidMotionInterface::KickMessage *>(msg_action);
			if (motion_task_) {
				motion_task_->exitTask();
			}
			motion_task_.reset(new NaoQiMotionKickTask(almotion_, msg->leg()));
			thread_pool_->enqueue(motion_task_);

			hummot_if_->set_msgid(msg->id());
		}

		msg_action->unref();
	} else {
		if (msg_walk_velocity) {
			if ((msg_walk_velocity->speed() < 0.) || (msg_walk_velocity->speed() > 1.)) {
				logger->log_warn(name(),
				                 "Walk velocity speed %f out of range [0.0..1.0],",
				                 "ignoring command",
				                 msg_walk_velocity->speed());
			} else {
				try {
					almotion_->setWalkTargetVelocity(msg_walk_velocity->x(),
					                                 msg_walk_velocity->y(),
					                                 msg_walk_velocity->theta(),
					                                 msg_walk_velocity->speed());
				} catch (AL::ALError &e) {
					logger->log_warn(name(), "WalkVelocity command failed: %s", e.what());
				}
			}
			hummot_if_->set_msgid(msg_walk_velocity->id());
			msg_walk_velocity->unref();
		}

		if (msg_move_head) {
			AL::ALValue names  = AL::ALValue::array("HeadYaw", "HeadPitch");
			AL::ALValue angles = AL::ALValue::array(msg_move_head->yaw(), msg_move_head->pitch());

			almotion_->setAngles(names, angles, msg_move_head->speed());
			msg_move_head->unref();
		}
	}
}
