
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
#include "goto_thread.h"
#include "openrave_thread.h"
#include "arm.h"

#include <interfaces/JacoInterface.h>
#include <interfaces/JacoBimanualInterface.h>
#include <utils/math/angle.h>
#include <core/threading/mutex.h>

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
  __dual_arms = arms;
  __final_mutex = NULL;
  __final = true;
}


/** Destructor. */
JacoBimanualGotoThread::~JacoBimanualGotoThread()
{
}

void
JacoBimanualGotoThread::init()
{
  __arms.l.arm = __dual_arms->left;
  __arms.r.arm = __dual_arms->right;

  __final_mutex = new Mutex();
  __v_arms[0] = &(__arms.l);
  __v_arms[1] = &(__arms.r);
}

void
JacoBimanualGotoThread::finalize()
{
  __dual_arms = NULL;

  __v_arms[0] = NULL;
  __v_arms[1] = NULL;

  __arms.l.arm = NULL;
  __arms.r.arm = NULL;

  delete __final_mutex;
  __final_mutex = NULL;
}

/** The main loop of this thread.
 * @see JacoGotoThread::loop
 */
void
JacoBimanualGotoThread::loop()
{
  __final_mutex->lock();
  bool final = __final;
  __final_mutex->unlock();

  if( __arms.l.arm == NULL || __arms.r.arm == NULL || !final ) {
    usleep(30e3);
    return;
  }

 // Current targets have been processed. Unref, if still refed
  if(__arms.l.target && __arms.r.target) {
    __arms.l.target.clear();
    __arms.r.target.clear();
    // trajectories hav been processed. remove those targets from queues.
    // This will automatically delete the trajectories as well as soon
    // as we leave this block (thanks to refptr)
    _lock_queues();
    __arms.l.arm->target_queue->pop_front();
    __arms.r.arm->target_queue->pop_front();
    _unlock_queues();
  }

  // Check for new targets
  _lock_queues();
  if( !__arms.l.arm->target_queue->empty() && !__arms.r.arm->target_queue->empty() ) {
    // get RefPtr to first target in queue
    __arms.l.target = __arms.l.arm->target_queue->front();
    __arms.r.target = __arms.r.arm->target_queue->front();
  }
  _unlock_queues();

  if( !__arms.l.target || !__arms.r.target || !__arms.l.target->coord || !__arms.r.target->coord) {
    //no new target in queue, or at least one target is not meant for
    // coordinated manipulation
    __arms.l.target.clear();
    __arms.r.target.clear();
    usleep(30e3);
    return;
  }

  if( __arms.l.target->type != __arms.r.target->type ) {
    logger->log_debug(name(), "target type mismatch, %i != %i", __arms.l.target->type, __arms.r.target->type);
    __arms.l.target.clear();
    __arms.r.target.clear();
    usleep(30e3);
    return;
  }

  if( __arms.l.target->trajec_state == TRAJEC_IK_ERROR
   || __arms.r.target->trajec_state == TRAJEC_IK_ERROR
   || __arms.l.target->trajec_state == TRAJEC_PLANNING_ERROR
   || __arms.r.target->trajec_state == TRAJEC_PLANNING_ERROR ) {
      logger->log_warn(name(), "Trajectory could not be planned. Abort!");
    // stop the current target and empty remaining queue, with appropriate error_code. This also sets "final" to true.
    __dual_arms->iface->set_error_code( __arms.l.target->trajec_state );
    stop();
    return;
  }

  if( __arms.l.target->trajec_state != __arms.r.target->trajec_state ) {
    logger->log_debug(name(), "trajec state mismatch, %i != %i", __arms.l.target->trajec_state, __arms.r.target->trajec_state);
    __arms.l.target.clear();
    __arms.r.target.clear();
    usleep(30e3);
    return;
  }

  switch( __arms.l.target->trajec_state ) {
    case TRAJEC_SKIP:
      // "regular" target. For now, we just process "GRIPPER", therefore do not
      //  change plotting
      logger->log_debug(name(), "No planning for these targets. Process, using current finger positions...");

      if(__arms.l.target->type != TARGET_GRIPPER) {
        logger->log_warn(name(), "Unknown target type %i, cannot process without planning!", __arms.l.target->type);
        stop();
        __dual_arms->iface->set_error_code( JacoInterface::ERROR_UNSPECIFIC );
      } else {
        _move_grippers();
        logger->log_debug(name(), "...targets processed");
      }
      break;

    case TRAJEC_READY:
      //logger->log_debug(name(), "Trajectories ready! Processing now.");
      // update trajectory state
      _lock_queues();
      __arms.l.target->trajec_state = TRAJEC_EXECUTING;
      __arms.r.target->trajec_state = TRAJEC_EXECUTING;
      _unlock_queues();

      // process trajectories only if it actually "exists"
      if( !__arms.l.target->trajec->empty() && !__arms.r.target->trajec->empty() ) {
        // first let the openrave_thread show the trajectory in the viewer
        __arms.l.arm->openrave_thread->plot_first();
        __arms.r.arm->openrave_thread->plot_first();

        // enable plotting of current positions
        __arms.l.arm->openrave_thread->plot_current(true);
        __arms.r.arm->openrave_thread->plot_current(true);

        // then execute the trajectories
        _exec_trajecs();
      }

      break;

    default:
      //logger->log_debug(name(), "Target is trajectory, but not ready yet!");
      __arms.l.target.clear();
      __arms.r.target.clear();
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
  // Check if any movement has startet (__final would be false then)
  __final_mutex->lock();
  bool final = __final;
  __final_mutex->unlock();
  if( !final ) {
    // There was some movement initiated. Check if it has finished
    _check_final();
    __final_mutex->lock();
    final = __final;
    __final_mutex->unlock();
  }

  if( !final )
    return false; // still moving

  // arm is not moving right now. Check if all targets have been processed
  _lock_queues();
  final = __arms.l.arm->target_queue->empty() && __arms.r.arm->target_queue->empty();
  _unlock_queues();

  return final;
}

/** Stops the current movement.
 * This also stops any currently enqueued motion.
 */
void
JacoBimanualGotoThread::stop()
{
  __arms.l.arm->goto_thread->stop();
  __arms.r.arm->goto_thread->stop();

  __arms.l.target.clear();
  __arms.r.target.clear();

  __final_mutex->lock();
  __final = true;
  __final_mutex->unlock();
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
JacoBimanualGotoThread::move_gripper(float l_f1, float l_f2, float l_f3, float r_f1, float r_f2, float r_f3)
{
  RefPtr<jaco_target_t> target_l(new jaco_target_t());
  RefPtr<jaco_target_t> target_r(new jaco_target_t());
  target_l->type = TARGET_GRIPPER;
  target_r->type = TARGET_GRIPPER;
  target_l->trajec_state = TRAJEC_SKIP;
  target_r->trajec_state = TRAJEC_SKIP;
  target_l->coord=true;
  target_r->coord=true;

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
  __arms.l.arm->target_mutex->lock();
  __arms.r.arm->target_mutex->lock();
}

inline void
JacoBimanualGotoThread::_unlock_queues() const
{
  __arms.l.arm->target_mutex->unlock();
  __arms.r.arm->target_mutex->unlock();
}

inline void
JacoBimanualGotoThread::_enqueue_targets(RefPtr<jaco_target_t> l, RefPtr<jaco_target_t> r)
{
  __arms.l.arm->target_queue->push_back(l);
  __arms.r.arm->target_queue->push_back(r);
}

void
JacoBimanualGotoThread::_move_grippers()
{
  __final_mutex->lock();
  __final = false;
  __final_mutex->unlock();

  for(unsigned int i=0; i<2; ++i) {
    __v_arms[i]->finger_last[0] = __v_arms[i]->arm->iface->finger1();
    __v_arms[i]->finger_last[1] = __v_arms[i]->arm->iface->finger2();
    __v_arms[i]->finger_last[2] = __v_arms[i]->arm->iface->finger3();
    __v_arms[i]->finger_last[3] = 0; // counter
  }

  // process new target
  try {
    // only fingers moving. use current joint values for that
    // we do this here and not in "move_gripper()" because we enqueue values. This ensures
    // that we move the gripper with the current joint values, not with the ones we had
    // when the target was enqueued!
    for(unsigned int i=0; i<2; ++i) {
      __v_arms[i]->target->pos.clear(); // just in case; should be empty anyway
      __v_arms[i]->target->pos.push_back(__v_arms[i]->arm->iface->joints(0));
      __v_arms[i]->target->pos.push_back(__v_arms[i]->arm->iface->joints(1));
      __v_arms[i]->target->pos.push_back(__v_arms[i]->arm->iface->joints(2));
      __v_arms[i]->target->pos.push_back(__v_arms[i]->arm->iface->joints(3));
      __v_arms[i]->target->pos.push_back(__v_arms[i]->arm->iface->joints(4));
      __v_arms[i]->target->pos.push_back(__v_arms[i]->arm->iface->joints(5));
      __v_arms[i]->target->type = TARGET_ANGULAR;
    }

    // just send the messages to the arm. nothing special here
    __arms.l.arm->arm->goto_joints(__arms.l.target->pos, __arms.l.target->fingers);
    __arms.r.arm->arm->goto_joints(__arms.r.target->pos, __arms.r.target->fingers);

  } catch( Exception &e ) {
    logger->log_warn(name(), "Error sending commands to arm. Ex:%s", e.what_no_backtrace());
  }
}

void
JacoBimanualGotoThread::_exec_trajecs()
{
  __final_mutex->lock();
  __final = false;
  __final_mutex->unlock();

  for(unsigned int i=0; i<2; ++i) {
    if( __v_arms[i]->target->fingers.empty() ) {
      // have no finger values. use current ones
      __v_arms[i]->target->fingers.push_back(__v_arms[i]->arm->iface->finger1());
      __v_arms[i]->target->fingers.push_back(__v_arms[i]->arm->iface->finger2());
      __v_arms[i]->target->fingers.push_back(__v_arms[i]->arm->iface->finger3());
    }
  }

  try {
     // stop old movement, if there was any
    __arms.l.arm->arm->stop();
    __arms.r.arm->arm->stop();

    // execute the trajectories
    logger->log_debug(name(), "exec traj: send traj commands...");

    // find out which arm has the shorter trajectory
    unsigned int first = 0;
    unsigned int second = 1;
    if( __v_arms[1]->target->trajec->size() < __v_arms[0]->target->trajec->size() ) {
      first = 1;
      second = 0;
    }
    JacoArm* arm_first  = __v_arms[first]->arm->arm;
    JacoArm* arm_second = __v_arms[second]->arm->arm;
    jaco_trajec_t* trajec_first  = *(__v_arms[first]->target->trajec);
    jaco_trajec_t* trajec_second = *(__v_arms[second]->target->trajec);
    unsigned int size_first =  trajec_first->size();
    unsigned int size_second = trajec_second->size();

    unsigned int it = 1; // iterator for the trajectories

    // send current position as initial trajec-point to arms
    for(unsigned int i=0; i<2; ++i) {
      jaco_trajec_point_t cur;
      cur.push_back( __v_arms[i]->arm->iface->joints(0) );
      cur.push_back( __v_arms[i]->arm->iface->joints(1) );
      cur.push_back( __v_arms[i]->arm->iface->joints(2) );
      cur.push_back( __v_arms[i]->arm->iface->joints(3) );
      cur.push_back( __v_arms[i]->arm->iface->joints(4) );
      cur.push_back( __v_arms[i]->arm->iface->joints(5) );
      __v_arms[i]->arm->arm->goto_joints(cur, __v_arms[i]->target->fingers, /*followup=*/false);
    }

    // send rest of trajectory as followup trajectory points.
    // Make sure to send the trajectory points alternatingly to the arm's
    // internal FIFO trajectory queue.
    while(it < size_first) {
      arm_first->goto_joints(trajec_first->at(it), __v_arms[first]->target->fingers, /*followup=*/true);
      arm_second->goto_joints(trajec_second->at(it), __v_arms[second]->target->fingers, /*followup=*/true);
      ++it;
    }

    // continue sending the rest of the longer trajectory
    while(it < size_second) {
      arm_second->goto_joints(trajec_second->at(it), __v_arms[second]->target->fingers, /*followup=*/true);
      ++it;
    }

    logger->log_debug(name(), "exec traj: ... DONE");

  } catch( Exception &e ) {
    logger->log_warn(name(), "Error executing trajectory. Ex:%s", e.what_no_backtrace());
  }
}

void
JacoBimanualGotoThread::_check_final()
{
  bool final = true;

  //logger->log_debug(name(), "check final");
  for(unsigned int i=0; i<2; ++i) {
    switch( __v_arms[i]->target->type ) {
      case TARGET_ANGULAR:
        //logger->log_debug(name(), "check[%u] final for TARGET ANGULAR", i);
        for( unsigned int j=0; j<6; ++j ) {
          final &= angle_distance(deg2rad(__v_arms[i]->target->pos.at(j)),
                                  deg2rad(__v_arms[i]->arm->iface->joints(j))) < 0.05;
        }
        break;

      case TARGET_GRIPPER:
        //logger->log_debug(name(), "check[%u] final for TARGET GRIPPER", i);
        final &= __v_arms[i]->arm->arm->final();
        break;

      default:
        //logger->log_debug(name(), "check[%u] final for UNKNOWN!!!", i);
        final &= false;
        break;
    }

    //logger->log_debug(name(), "check[%u] final (joints): %u", i, final);
  }


  if( final ) {
    // check fingeres
    for(unsigned int i=0; i<2; ++i) {
      //logger->log_debug(name(), "check[%u] fingers for final", i);

      if( __v_arms[i]->finger_last[0] == __v_arms[i]->arm->iface->finger1() &&
          __v_arms[i]->finger_last[1] == __v_arms[i]->arm->iface->finger2() &&
          __v_arms[i]->finger_last[2] == __v_arms[i]->arm->iface->finger3() ) {
        __v_arms[i]->finger_last[3] += 1;
      } else {
        __v_arms[i]->finger_last[0] = __v_arms[i]->arm->iface->finger1();
        __v_arms[i]->finger_last[1] = __v_arms[i]->arm->iface->finger2();
        __v_arms[i]->finger_last[2] = __v_arms[i]->arm->iface->finger3();
        __v_arms[i]->finger_last[3] = 0; // counter
      }
      final &= __v_arms[i]->finger_last[3] > 10;
      //logger->log_debug(name(), "check[%u] final (all): %u", i, final);
    }
  }

  __final_mutex->lock();
  __final = final;
  __final_mutex->unlock();
}
