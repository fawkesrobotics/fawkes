
/***************************************************************************
 *  goto_thread.cpp - Kinova Jaco plugin movement thread
 *
 *  Created: Thu Jun 20 15:04:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#include "goto_thread.h"

#include "arm.h"
#include "openrave_thread.h"

#include <core/threading/mutex.h>
#include <interfaces/JacoInterface.h>
#include <utils/math/angle.h>

#include <unistd.h>

using namespace fawkes;

/** @class JacoGotoThread "goto_thread.h"
 * Jaco Arm movement thread.
 * This thread handles the movement of the arm.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param name thread name
 * @param arm pointer to jaco_arm_t struct, to be used in this thread
 */
JacoGotoThread::JacoGotoThread(const char *name, jaco_arm_t *arm)
: Thread(name, Thread::OPMODE_CONTINUOUS)
{
	arm_         = arm;
	final_mutex_ = NULL;

	final_ = true;

	wait_status_check_ = 0; //wait loops to check for jaco_retract_mode_t again
}

/** Destructor. */
JacoGotoThread::~JacoGotoThread()
{
}

/** Initialize. */
void
JacoGotoThread::init()
{
	final_mutex_ = new Mutex();
}

/** Finalize. */
void
JacoGotoThread::finalize()
{
	delete final_mutex_;
	final_mutex_ = NULL;
	arm_         = NULL;
}

/** Check if arm is final.
 * Checks if the movements started by this thread have finished, and
 * if the target_queue has been fully processed. Otherwise the arm
 * is probably still moving and therefore not final.
 *
 * @return "true" if arm is not moving anymore, "false" otherwise
 */
bool
JacoGotoThread::final()
{
	// Check if any movement has startet (final_ would be false then)
	final_mutex_->lock();
	bool final = final_;
	final_mutex_->unlock();
	if (!final) {
		// There was some movement initiated. Check if it has finished
		_check_final();
		final_mutex_->lock();
		final = final_;
		final_mutex_->unlock();
	}

	if (!final)
		return false; // still moving

	// arm is not moving right now. Check if all targets have been processed
	arm_->target_mutex->lock();
	final = arm_->target_queue->empty();
	arm_->target_mutex->unlock();

	if (final)
		arm_->openrave_thread->plot_current(false);

	return final;
}

/** Set new target, given cartesian coordinates.
 * This target is added to the queue, skipping trajectory planning.
 * CAUTION: This also means: no collision avoidance!
 *
 * @param x x-coordinate of target position
 * @param y y-coordinate of target position
 * @param z z-coordinate of target position
 * @param e1 1st euler rotation of target orientation
 * @param e2 2nd euler rotation of target orientation
 * @param e3 3rd euler rotation of target orientation
 * @param f1 value of 1st finger
 * @param f2 value of 2nd finger
 * @param f3 value of 3rd finger
 */
void
JacoGotoThread::set_target(float x,
                           float y,
                           float z,
                           float e1,
                           float e2,
                           float e3,
                           float f1,
                           float f2,
                           float f3)
{
	RefPtr<jaco_target_t> target(new jaco_target_t());
	target->type         = TARGET_CARTESIAN;
	target->trajec_state = TRAJEC_SKIP;
	target->coord        = false;

	target->pos.push_back(x);
	target->pos.push_back(y);
	target->pos.push_back(z);
	target->pos.push_back(e1);
	target->pos.push_back(e2);
	target->pos.push_back(e3);

	if (f1 > 0.f && f2 > 0.f && f3 > 0.f) {
		target->fingers.push_back(f1);
		target->fingers.push_back(f2);
		target->fingers.push_back(f3);
	}
	arm_->target_mutex->lock();
	arm_->target_queue->push_back(target);
	arm_->target_mutex->unlock();
}

/** Set new target, given joint positions
 * This target is added to the queue, skipping trajectory planning.
 * CAUTION: This also means: no collision avoidance!
 *
 * @param j1 angular position of 1st joint (in degree)
 * @param j2 angular position of 2nd joint (in degree)
 * @param j3 angular position of 3rd joint (in degree)
 * @param j4 angular position of 4th joint (in degree)
 * @param j5 angular position of 5th joint (in degree)
 * @param j6 angular position of 6th joint (in degree)
 * @param f1 value of 1st finger
 * @param f2 value of 2nd finger
 * @param f3 value of 3rd finger
 */
void
JacoGotoThread::set_target_ang(float j1,
                               float j2,
                               float j3,
                               float j4,
                               float j5,
                               float j6,
                               float f1,
                               float f2,
                               float f3)
{
	RefPtr<jaco_target_t> target(new jaco_target_t());
	target->type         = TARGET_ANGULAR;
	target->trajec_state = TRAJEC_SKIP;
	target->coord        = false;

	target->pos.push_back(j1);
	target->pos.push_back(j2);
	target->pos.push_back(j3);
	target->pos.push_back(j4);
	target->pos.push_back(j5);
	target->pos.push_back(j6);

	if (f1 > 0.f && f2 > 0.f && f3 > 0.f) {
		target->fingers.push_back(f1);
		target->fingers.push_back(f2);
		target->fingers.push_back(f3);
	}
	arm_->target_mutex->lock();
	arm_->target_queue->push_back(target);
	arm_->target_mutex->unlock();
}

/** Moves the arm to the "READY" position.
 * This target is added to the queue, skipping trajectory planning.
 * CAUTION: This also means: no collision avoidance!
 */
void
JacoGotoThread::pos_ready()
{
	RefPtr<jaco_target_t> target(new jaco_target_t());
	target->type = TARGET_READY;
	arm_->target_mutex->lock();
	arm_->target_queue->push_back(target);
	arm_->target_mutex->unlock();
}

/** Moves the arm to the "RETRACT" position.
 * This target is added to the queue, skipping trajectory planning.
 * CAUTION: This also means: no collision avoidance!
 */
void
JacoGotoThread::pos_retract()
{
	RefPtr<jaco_target_t> target(new jaco_target_t());
	target->type = TARGET_RETRACT;
	arm_->target_mutex->lock();
	arm_->target_queue->push_back(target);
	arm_->target_mutex->unlock();
}

/** Moves only the gripper.
 * @param f1 value of 1st finger
 * @param f2 value of 2nd finger
 * @param f3 value of 3rd finger
 */
void
JacoGotoThread::move_gripper(float f1, float f2, float f3)
{
	RefPtr<jaco_target_t> target(new jaco_target_t());
	target->type = TARGET_GRIPPER;

	target->fingers.push_back(f1);
	target->fingers.push_back(f2);
	target->fingers.push_back(f3);

	arm_->target_mutex->lock();
	arm_->target_queue->push_back(target);
	arm_->target_mutex->unlock();
}

/** Stops the current movement.
 * This also stops any currently enqueued motion.
 */
void
JacoGotoThread::stop()
{
	try {
		arm_->arm->stop();

		arm_->target_mutex->lock();
		arm_->target_queue->clear();
		arm_->target_mutex->unlock();

		target_.clear();

		final_mutex_->lock();
		final_ = true;
		final_mutex_->unlock();

	} catch (Exception &e) {
		logger->log_warn(name(), "Error sending stop command to arm. Ex:%s", e.what());
	}
}

/** The main loop of this thread.
 * It gets the first target in the target_queue and checks if it is ready
 * to be processed. The target is removed from the queue if the movement is
 * "final" (see final()), which is when this method starts processing
 * the queue again.
 */
void
JacoGotoThread::loop()
{
	final_mutex_->lock();
	bool final = final_;
	final_mutex_->unlock();

	if (arm_ == NULL || arm_->arm == NULL || !final) {
		usleep(30e3);
		return;
	}

	// Current target has been processed. Unref, if still refed
	if (target_) {
		target_.clear();
		// trajectory has been processed. remove that target from queue.
		// This will automatically delete the trajectory as well as soon
		// as we leave this block (thanks to refptr)
		arm_->target_mutex->lock();
		arm_->target_queue->pop_front();
		arm_->target_mutex->unlock();
	}

	// Check for new targets
	arm_->target_mutex->lock();
	if (!arm_->target_queue->empty()) {
		// get RefPtr to first target in queue
		target_ = arm_->target_queue->front();
	}
	arm_->target_mutex->unlock();
	if (!target_ || target_->coord) {
		//no new target in queue, or target needs coordination of both arms,
		// which is not what this thread does
		target_.clear();
		usleep(30e3);
		return;
	}

	switch (target_->trajec_state) {
	case TRAJEC_SKIP:
		// "regular" target
		logger->log_debug(
		  name(), "No planning for this new target. Process, using current finger positions...");

		if (target_->type != TARGET_GRIPPER) {
			// arm moves! clear previously drawn trajectory plot
			arm_->openrave_thread->plot_first();

			// also enable ploting current joint positions
			arm_->openrave_thread->plot_current(true);
		}
		_goto_target();
		logger->log_debug(name(), "...target processed");
		break;

	case TRAJEC_READY:
		logger->log_debug(name(), "Trajectory ready! Processing now.");
		// update trajectory state
		arm_->target_mutex->lock();
		target_->trajec_state = TRAJEC_EXECUTING;
		arm_->target_mutex->unlock();

		// process trajectory only if it actually "exists"
		if (!target_->trajec->empty()) {
			// first let the openrave_thread show the trajectory in the viewer
			arm_->openrave_thread->plot_first();

			// enable plotting current positions
			arm_->openrave_thread->plot_current(true);

			// then execute the trajectory
			_exec_trajec(*(target_->trajec));
		}
		break;

	case TRAJEC_PLANNING_ERROR:
		logger->log_debug(name(), "Trajectory could not be planned. Abort!");
		// stop the current and remaining queue, with appropriate error_code. This also sets "final" to true.
		stop();
		arm_->iface->set_error_code(JacoInterface::ERROR_PLANNING);
		break;

	default:
		//logger->log_debug("Target is trajectory, but not ready yet!");
		target_.clear();
		usleep(30e3);
		break;
	}
}

/* PRIVATE METHODS */
void
JacoGotoThread::_check_final()
{
	bool check_fingers = false;
	bool final         = true;

	//logger->log_debug(name(), "check final");
	switch (target_->type) {
	case TARGET_READY:
	case TARGET_RETRACT:
		if (wait_status_check_ == 0) {
			//logger->log_debug(name(), "check final for TARGET_MODE");
			final_mutex_->lock();
			final_ = arm_->arm->final();
			final_mutex_->unlock();
		} else if (wait_status_check_ >= 10) {
			wait_status_check_ = 0;
		} else {
			++wait_status_check_;
		}
		break;

	case TARGET_ANGULAR:
		//logger->log_debug(name(), "check final for TARGET ANGULAR");
		//final = arm_->arm->final();
		for (unsigned int i = 0; i < 6; ++i) {
			final &=
			  (angle_distance(deg2rad(target_->pos.at(i)), deg2rad(arm_->iface->joints(i))) < 0.05);
		}
		final_mutex_->lock();
		final_ = final;
		final_mutex_->unlock();
		check_fingers = true;
		break;

	case TARGET_GRIPPER:
		//logger->log_debug(name(), "check final for TARGET GRIPPER");
		final_mutex_->lock();
		final_ = arm_->arm->final();
		final_mutex_->unlock();
		check_fingers = true;
		break;

	default: //TARGET_CARTESIAN
		//logger->log_debug(name(), "check final for TARGET CARTESIAN");
		//final = arm_->arm->final();
		final &= (angle_distance(target_->pos.at(0), arm_->iface->x()) < 0.01);
		final &= (angle_distance(target_->pos.at(1), arm_->iface->y()) < 0.01);
		final &= (angle_distance(target_->pos.at(2), arm_->iface->z()) < 0.01);
		final &= (angle_distance(target_->pos.at(3), arm_->iface->euler1()) < 0.1);
		final &= (angle_distance(target_->pos.at(4), arm_->iface->euler2()) < 0.1);
		final &= (angle_distance(target_->pos.at(5), arm_->iface->euler3()) < 0.1);
		final_mutex_->lock();
		final_ = final;
		final_mutex_->unlock();

		check_fingers = true;
		break;
	}

	final_mutex_->lock();
	final = final_;
	final_mutex_->unlock();
	//logger->log_debug(name(), "check final: %u", final);

	if (check_fingers && final) {
		//logger->log_debug(name(), "check fingeres for final");

		// also check fingeres
		if (finger_last_[0] == arm_->iface->finger1() && finger_last_[1] == arm_->iface->finger2()
		    && finger_last_[2] == arm_->iface->finger3()) {
			finger_last_[3] += 1;
		} else {
			finger_last_[0] = arm_->iface->finger1();
			finger_last_[1] = arm_->iface->finger2();
			finger_last_[2] = arm_->iface->finger3();
			finger_last_[3] = 0; // counter
		}
		final_mutex_->lock();
		final_ &= finger_last_[3] > 10;
		final_mutex_->unlock();
	}
}

void
JacoGotoThread::_goto_target()
{
	finger_last_[0] = arm_->iface->finger1();
	finger_last_[1] = arm_->iface->finger2();
	finger_last_[2] = arm_->iface->finger3();
	finger_last_[3] = 0; // counter

	final_mutex_->lock();
	final_ = false;
	final_mutex_->unlock();

	// process new target
	try {
		arm_->arm->stop(); // stop old movement, if there was any
		                   //arm_->arm->start_api_ctrl();

		if (target_->type == TARGET_GRIPPER) {
			// only fingers moving. use current joint values for that
			// we do this here and not in "move_gripper()" because we enqueue values. This ensures
			// that we move the gripper with the current joint values, not with the ones we had
			// when the target was enqueued!
			target_->pos.clear(); // just in case; should be empty anyway
			target_->pos.push_back(arm_->iface->joints(0));
			target_->pos.push_back(arm_->iface->joints(1));
			target_->pos.push_back(arm_->iface->joints(2));
			target_->pos.push_back(arm_->iface->joints(3));
			target_->pos.push_back(arm_->iface->joints(4));
			target_->pos.push_back(arm_->iface->joints(5));
			target_->type = TARGET_ANGULAR;
		}

		switch (target_->type) {
		case TARGET_ANGULAR:
			logger->log_debug(name(), "target_type: TARGET_ANGULAR");
			if (target_->fingers.empty()) {
				// have no finger values. use current ones
				target_->fingers.push_back(arm_->iface->finger1());
				target_->fingers.push_back(arm_->iface->finger2());
				target_->fingers.push_back(arm_->iface->finger3());
			}
			arm_->arm->goto_joints(target_->pos, target_->fingers);
			break;

		case TARGET_READY:
			logger->log_debug(name(), "loop: target_type: TARGET_READY");
			wait_status_check_ = 0;
			arm_->arm->goto_ready();
			break;

		case TARGET_RETRACT:
			logger->log_debug(name(), "target_type: TARGET_RETRACT");
			wait_status_check_ = 0;
			arm_->arm->goto_retract();
			break;

		default: //TARGET_CARTESIAN
			logger->log_debug(name(), "target_type: TARGET_CARTESIAN");
			if (target_->fingers.empty()) {
				// have no finger values. use current ones
				target_->fingers.push_back(arm_->iface->finger1());
				target_->fingers.push_back(arm_->iface->finger2());
				target_->fingers.push_back(arm_->iface->finger3());
			}
			arm_->arm->goto_coords(target_->pos, target_->fingers);
			break;
		}

	} catch (Exception &e) {
		logger->log_warn(name(), "Error sending command to arm. Ex:%s", e.what_no_backtrace());
	}
}

void
JacoGotoThread::_exec_trajec(jaco_trajec_t *trajec)
{
	final_mutex_->lock();
	final_ = false;
	final_mutex_->unlock();

	if (target_->fingers.empty()) {
		// have no finger values. use current ones
		target_->fingers.push_back(arm_->iface->finger1());
		target_->fingers.push_back(arm_->iface->finger2());
		target_->fingers.push_back(arm_->iface->finger3());
	}

	try {
		// stop old movement
		arm_->arm->stop();

		// execute the trajectory
		logger->log_debug(name(), "exec traj: send traj commands...");
		arm_->arm->goto_trajec(trajec, target_->fingers);
		logger->log_debug(name(), "exec traj: ... DONE");

	} catch (Exception &e) {
		logger->log_warn(name(), "Error executing trajectory. Ex:%s", e.what_no_backtrace());
	}
}
