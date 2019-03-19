
/***************************************************************************
 *  bimaual_goto_thread.cpp - Jaco plugin movement thread for coordinated bimanual manipulation
 *
 *  Created: Mon Sep 29 23:17:12 2014
 *  Copyright  2014  Bahram Maleki-Fard
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

#include "bimanual_goto_thread.h"

#include "arm.h"
#include "goto_thread.h"
#include "openrave_thread.h"

#include <core/threading/mutex.h>
#include <interfaces/JacoBimanualInterface.h>
#include <interfaces/JacoInterface.h>
#include <utils/math/angle.h>

#include <unistd.h>

using namespace fawkes;

/** @class JacoBimanualGotoThread "bimanual_goto_thread.h"
 * Jaco Arm movement thread.
 * This thread handles the movement of the arm.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param arms pointer to jaco_dual_arm_t struct, to be used in this thread
 */
JacoBimanualGotoThread::JacoBimanualGotoThread(jaco_dual_arm_t *arms)
: Thread("JacoBimanualGotoThread", Thread::OPMODE_CONTINUOUS)
{
	dual_arms_   = arms;
	final_mutex_ = NULL;
	final_       = true;
}

/** Destructor. */
JacoBimanualGotoThread::~JacoBimanualGotoThread()
{
}

void
JacoBimanualGotoThread::init()
{
	arms_.l.arm = dual_arms_->left;
	arms_.r.arm = dual_arms_->right;

	final_mutex_ = new Mutex();
	v_arms_[0]   = &(arms_.l);
	v_arms_[1]   = &(arms_.r);
}

void
JacoBimanualGotoThread::finalize()
{
	dual_arms_ = NULL;

	v_arms_[0] = NULL;
	v_arms_[1] = NULL;

	arms_.l.arm = NULL;
	arms_.r.arm = NULL;

	delete final_mutex_;
	final_mutex_ = NULL;
}

/** The main loop of this thread.
 * @see JacoGotoThread::loop
 */
void
JacoBimanualGotoThread::loop()
{
	final_mutex_->lock();
	bool final = final_;
	final_mutex_->unlock();

	if (arms_.l.arm == NULL || arms_.r.arm == NULL || !final) {
		usleep(30e3);
		return;
	}

	// Current targets have been processed. Unref, if still refed
	if (arms_.l.target && arms_.r.target) {
		arms_.l.target.clear();
		arms_.r.target.clear();
		// trajectories hav been processed. remove those targets from queues.
		// This will automatically delete the trajectories as well as soon
		// as we leave this block (thanks to refptr)
		_lock_queues();
		arms_.l.arm->target_queue->pop_front();
		arms_.r.arm->target_queue->pop_front();
		_unlock_queues();
	}

	// Check for new targets
	_lock_queues();
	if (!arms_.l.arm->target_queue->empty() && !arms_.r.arm->target_queue->empty()) {
		// get RefPtr to first target in queue
		arms_.l.target = arms_.l.arm->target_queue->front();
		arms_.r.target = arms_.r.arm->target_queue->front();
	}
	_unlock_queues();

	if (!arms_.l.target || !arms_.r.target || !arms_.l.target->coord || !arms_.r.target->coord) {
		//no new target in queue, or at least one target is not meant for
		// coordinated manipulation
		arms_.l.target.clear();
		arms_.r.target.clear();
		usleep(30e3);
		return;
	}

	if (arms_.l.target->type != arms_.r.target->type) {
		logger->log_debug(name(),
		                  "target type mismatch, %i != %i",
		                  arms_.l.target->type,
		                  arms_.r.target->type);
		arms_.l.target.clear();
		arms_.r.target.clear();
		usleep(30e3);
		return;
	}

	if (arms_.l.target->trajec_state == TRAJEC_IK_ERROR
	    || arms_.r.target->trajec_state == TRAJEC_IK_ERROR
	    || arms_.l.target->trajec_state == TRAJEC_PLANNING_ERROR
	    || arms_.r.target->trajec_state == TRAJEC_PLANNING_ERROR) {
		logger->log_warn(name(), "Trajectory could not be planned. Abort!");
		// stop the current target and empty remaining queue, with appropriate error_code. This also sets "final" to true.
		dual_arms_->iface->set_error_code(arms_.l.target->trajec_state);
		stop();
		return;
	}

	if (arms_.l.target->trajec_state != arms_.r.target->trajec_state) {
		logger->log_debug(name(),
		                  "trajec state mismatch, %i != %i",
		                  arms_.l.target->trajec_state,
		                  arms_.r.target->trajec_state);
		arms_.l.target.clear();
		arms_.r.target.clear();
		usleep(30e3);
		return;
	}

	switch (arms_.l.target->trajec_state) {
	case TRAJEC_SKIP:
		// "regular" target. For now, we just process "GRIPPER", therefore do not
		//  change plotting
		logger->log_debug(name(),
		                  "No planning for these targets. Process, using current finger positions...");

		if (arms_.l.target->type != TARGET_GRIPPER) {
			logger->log_warn(name(),
			                 "Unknown target type %i, cannot process without planning!",
			                 arms_.l.target->type);
			stop();
			dual_arms_->iface->set_error_code(JacoInterface::ERROR_UNSPECIFIC);
		} else {
			_move_grippers();
			logger->log_debug(name(), "...targets processed");
		}
		break;

	case TRAJEC_READY:
		//logger->log_debug(name(), "Trajectories ready! Processing now.");
		// update trajectory state
		_lock_queues();
		arms_.l.target->trajec_state = TRAJEC_EXECUTING;
		arms_.r.target->trajec_state = TRAJEC_EXECUTING;
		_unlock_queues();

		// process trajectories only if it actually "exists"
		if (!arms_.l.target->trajec->empty() && !arms_.r.target->trajec->empty()) {
			// first let the openrave_thread show the trajectory in the viewer
			arms_.l.arm->openrave_thread->plot_first();
			arms_.r.arm->openrave_thread->plot_first();

			// enable plotting of current positions
			arms_.l.arm->openrave_thread->plot_current(true);
			arms_.r.arm->openrave_thread->plot_current(true);

			// then execute the trajectories
			_exec_trajecs();
		}

		break;

	default:
		//logger->log_debug(name(), "Target is trajectory, but not ready yet!");
		arms_.l.target.clear();
		arms_.r.target.clear();
		usleep(30e3);
		break;
	}
}

/** Check if arm is final.
 * @see JacoGotoThread::final
 * @return "true" if arm is not moving anymore, "false" otherwise
 */
bool
JacoBimanualGotoThread::final()
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
	_lock_queues();
	final = arms_.l.arm->target_queue->empty() && arms_.r.arm->target_queue->empty();
	_unlock_queues();

	return final;
}

/** Stops the current movement.
 * This also stops any currently enqueued motion.
 */
void
JacoBimanualGotoThread::stop()
{
	arms_.l.arm->goto_thread->stop();
	arms_.r.arm->goto_thread->stop();

	arms_.l.target.clear();
	arms_.r.target.clear();

	final_mutex_->lock();
	final_ = true;
	final_mutex_->unlock();
}

/** Moves only the gripper of both arms
 * @param l_f1 value of 1st finger of left arm
 * @param l_f2 value of 2nd finger of left arm
 * @param l_f3 value of 3rd finger of left arm
 * @param r_f1 value of 1st finger of right arm
 * @param r_f2 value of 2nd finger of right arm
 * @param r_f3 value of 3rd finger of right arm
 */
void
JacoBimanualGotoThread::move_gripper(float l_f1,
                                     float l_f2,
                                     float l_f3,
                                     float r_f1,
                                     float r_f2,
                                     float r_f3)
{
	RefPtr<jaco_target_t> target_l(new jaco_target_t());
	RefPtr<jaco_target_t> target_r(new jaco_target_t());
	target_l->type         = TARGET_GRIPPER;
	target_r->type         = TARGET_GRIPPER;
	target_l->trajec_state = TRAJEC_SKIP;
	target_r->trajec_state = TRAJEC_SKIP;
	target_l->coord        = true;
	target_r->coord        = true;

	target_l->fingers.push_back(l_f1);
	target_l->fingers.push_back(l_f2);
	target_l->fingers.push_back(l_f3);
	target_r->fingers.push_back(l_f1);
	target_r->fingers.push_back(l_f2);
	target_r->fingers.push_back(l_f3);

	_lock_queues();
	_enqueue_targets(target_l, target_r);
	_unlock_queues();
}

/* PRIVATE METHODS */
inline void
JacoBimanualGotoThread::_lock_queues() const
{
	arms_.l.arm->target_mutex->lock();
	arms_.r.arm->target_mutex->lock();
}

inline void
JacoBimanualGotoThread::_unlock_queues() const
{
	arms_.l.arm->target_mutex->unlock();
	arms_.r.arm->target_mutex->unlock();
}

inline void
JacoBimanualGotoThread::_enqueue_targets(RefPtr<jaco_target_t> l, RefPtr<jaco_target_t> r)
{
	arms_.l.arm->target_queue->push_back(l);
	arms_.r.arm->target_queue->push_back(r);
}

void
JacoBimanualGotoThread::_move_grippers()
{
	final_mutex_->lock();
	final_ = false;
	final_mutex_->unlock();

	for (unsigned int i = 0; i < 2; ++i) {
		v_arms_[i]->finger_last[0] = v_arms_[i]->arm->iface->finger1();
		v_arms_[i]->finger_last[1] = v_arms_[i]->arm->iface->finger2();
		v_arms_[i]->finger_last[2] = v_arms_[i]->arm->iface->finger3();
		v_arms_[i]->finger_last[3] = 0; // counter
	}

	// process new target
	try {
		// only fingers moving. use current joint values for that
		// we do this here and not in "move_gripper()" because we enqueue values. This ensures
		// that we move the gripper with the current joint values, not with the ones we had
		// when the target was enqueued!
		for (unsigned int i = 0; i < 2; ++i) {
			v_arms_[i]->target->pos.clear(); // just in case; should be empty anyway
			v_arms_[i]->target->pos.push_back(v_arms_[i]->arm->iface->joints(0));
			v_arms_[i]->target->pos.push_back(v_arms_[i]->arm->iface->joints(1));
			v_arms_[i]->target->pos.push_back(v_arms_[i]->arm->iface->joints(2));
			v_arms_[i]->target->pos.push_back(v_arms_[i]->arm->iface->joints(3));
			v_arms_[i]->target->pos.push_back(v_arms_[i]->arm->iface->joints(4));
			v_arms_[i]->target->pos.push_back(v_arms_[i]->arm->iface->joints(5));
			v_arms_[i]->target->type = TARGET_ANGULAR;
		}

		// just send the messages to the arm. nothing special here
		arms_.l.arm->arm->goto_joints(arms_.l.target->pos, arms_.l.target->fingers);
		arms_.r.arm->arm->goto_joints(arms_.r.target->pos, arms_.r.target->fingers);

	} catch (Exception &e) {
		logger->log_warn(name(), "Error sending commands to arm. Ex:%s", e.what_no_backtrace());
	}
}

void
JacoBimanualGotoThread::_exec_trajecs()
{
	final_mutex_->lock();
	final_ = false;
	final_mutex_->unlock();

	for (unsigned int i = 0; i < 2; ++i) {
		if (v_arms_[i]->target->fingers.empty()) {
			// have no finger values. use current ones
			v_arms_[i]->target->fingers.push_back(v_arms_[i]->arm->iface->finger1());
			v_arms_[i]->target->fingers.push_back(v_arms_[i]->arm->iface->finger2());
			v_arms_[i]->target->fingers.push_back(v_arms_[i]->arm->iface->finger3());
		}
	}

	try {
		// stop old movement, if there was any
		arms_.l.arm->arm->stop();
		arms_.r.arm->arm->stop();

		// execute the trajectories
		logger->log_debug(name(), "exec traj: send traj commands...");

		// find out which arm has the shorter trajectory
		unsigned int first  = 0;
		unsigned int second = 1;
		if (v_arms_[1]->target->trajec->size() < v_arms_[0]->target->trajec->size()) {
			first  = 1;
			second = 0;
		}
		JacoArm *      arm_first     = v_arms_[first]->arm->arm;
		JacoArm *      arm_second    = v_arms_[second]->arm->arm;
		jaco_trajec_t *trajec_first  = *(v_arms_[first]->target->trajec);
		jaco_trajec_t *trajec_second = *(v_arms_[second]->target->trajec);
		unsigned int   size_first    = trajec_first->size();
		unsigned int   size_second   = trajec_second->size();

		unsigned int it = 1; // iterator for the trajectories

		// send current position as initial trajec-point to arms
		for (unsigned int i = 0; i < 2; ++i) {
			jaco_trajec_point_t cur;
			cur.push_back(v_arms_[i]->arm->iface->joints(0));
			cur.push_back(v_arms_[i]->arm->iface->joints(1));
			cur.push_back(v_arms_[i]->arm->iface->joints(2));
			cur.push_back(v_arms_[i]->arm->iface->joints(3));
			cur.push_back(v_arms_[i]->arm->iface->joints(4));
			cur.push_back(v_arms_[i]->arm->iface->joints(5));
			v_arms_[i]->arm->arm->goto_joints(cur, v_arms_[i]->target->fingers, /*followup=*/false);
		}

		// send rest of trajectory as followup trajectory points.
		// Make sure to send the trajectory points alternatingly to the arm's
		// internal FIFO trajectory queue.
		while (it < size_first) {
			arm_first->goto_joints(trajec_first->at(it),
			                       v_arms_[first]->target->fingers,
			                       /*followup=*/true);
			arm_second->goto_joints(trajec_second->at(it),
			                        v_arms_[second]->target->fingers,
			                        /*followup=*/true);
			++it;
		}

		// continue sending the rest of the longer trajectory
		while (it < size_second) {
			arm_second->goto_joints(trajec_second->at(it),
			                        v_arms_[second]->target->fingers,
			                        /*followup=*/true);
			++it;
		}

		logger->log_debug(name(), "exec traj: ... DONE");

	} catch (Exception &e) {
		logger->log_warn(name(), "Error executing trajectory. Ex:%s", e.what_no_backtrace());
	}
}

void
JacoBimanualGotoThread::_check_final()
{
	bool final = true;

	//logger->log_debug(name(), "check final");
	for (unsigned int i = 0; i < 2; ++i) {
		switch (v_arms_[i]->target->type) {
		case TARGET_ANGULAR:
			//logger->log_debug(name(), "check[%u] final for TARGET ANGULAR", i);
			for (unsigned int j = 0; j < 6; ++j) {
				final &= angle_distance(deg2rad(v_arms_[i]->target->pos.at(j)),
				                        deg2rad(v_arms_[i]->arm->iface->joints(j)))
				         < 0.05;
			}
			break;

		case TARGET_GRIPPER:
			//logger->log_debug(name(), "check[%u] final for TARGET GRIPPER", i);
			final &= v_arms_[i]->arm->arm->final();
			break;

		default:
			//logger->log_debug(name(), "check[%u] final for UNKNOWN!!!", i);
			final &= false;
			break;
		}

		//logger->log_debug(name(), "check[%u] final (joints): %u", i, final);
	}

	if (final) {
		// check fingeres
		for (unsigned int i = 0; i < 2; ++i) {
			//logger->log_debug(name(), "check[%u] fingers for final", i);

			if (v_arms_[i]->finger_last[0] == v_arms_[i]->arm->iface->finger1()
			    && v_arms_[i]->finger_last[1] == v_arms_[i]->arm->iface->finger2()
			    && v_arms_[i]->finger_last[2] == v_arms_[i]->arm->iface->finger3()) {
				v_arms_[i]->finger_last[3] += 1;
			} else {
				v_arms_[i]->finger_last[0] = v_arms_[i]->arm->iface->finger1();
				v_arms_[i]->finger_last[1] = v_arms_[i]->arm->iface->finger2();
				v_arms_[i]->finger_last[2] = v_arms_[i]->arm->iface->finger3();
				v_arms_[i]->finger_last[3] = 0; // counter
			}
			final &= v_arms_[i]->finger_last[3] > 10;
			//logger->log_debug(name(), "check[%u] final (all): %u", i, final);
		}
	}

	final_mutex_->lock();
	final_ = final;
	final_mutex_->unlock();
}
