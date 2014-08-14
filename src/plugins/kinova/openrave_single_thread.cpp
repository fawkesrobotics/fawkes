
/***************************************************************************
 *  openrave_single_thread.cpp - Kinova plugin OpenRAVE Thread for single-arm setup
 *
 *  Created: Mon Jul 28 19:43:20 2014
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

#include "openrave_single_thread.h"
#include "arm.h"
#include "types.h"

#include <interfaces/JacoInterface.h>
#include <core/threading/mutex.h>

#include <cmath>
#include <stdio.h>
#include <cstring>

#ifdef HAVE_OPENRAVE
 #include <plugins/openrave/environment.h>
 #include <plugins/openrave/robot.h>
 #include <plugins/openrave/manipulator.h>
 #include <plugins/openrave/manipulators/kinova_jaco.h>

 using namespace OpenRAVE;
#endif

using namespace fawkes;
using namespace std;

/** @class KinovaOpenraveSingleThread "openrave_single_thread.h"
 * Jaco Arm thread for single-arm setup, integrating OpenRAVE
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaOpenraveSingleThread::KinovaOpenraveSingleThread(const char *manipname, bool load_robot)
  : KinovaOpenraveBaseThread("KinovaOpenraveSingleThread")
{
  __arm = NULL;
  __manipname = manipname;
  __load_robot = load_robot;
}

/** Constructor.
 * @param thread_name thread name
 */
KinovaOpenraveSingleThread::KinovaOpenraveSingleThread(const char *name, const char *manipname, bool load_robot)
  : KinovaOpenraveBaseThread(name)
{
  __arm = NULL;
  __manipname = manipname;
  __load_robot = load_robot;
}


void
KinovaOpenraveSingleThread::_load_robot()
{
#ifdef HAVE_OPENRAVE

  if(__load_robot) {
    __cfg_OR_robot_file    = config->get_string("/hardware/jaco/openrave/robot_file");

    try {
      __OR_robot = openrave->add_robot(__cfg_OR_robot_file, false);
    } catch (Exception& e) {
      throw fawkes::Exception("Could not add robot '%s' to openrave environment. (Error: %s)", __cfg_OR_robot_file.c_str(), e.what_no_backtrace());
    }

    try {
      __OR_manip = new OpenRaveManipulatorKinovaJaco(6, 6);
      __OR_manip->add_motor(0,0);
      __OR_manip->add_motor(1,1);
      __OR_manip->add_motor(2,2);
      __OR_manip->add_motor(3,3);
      __OR_manip->add_motor(4,4);
      __OR_manip->add_motor(5,5);

      // Set manipulator and offsets.
      openrave->set_manipulator(__OR_robot, __OR_manip, 0.f, 0.f, 0.f);

      if( __cfg_OR_auto_load_ik ) {
        openrave->get_environment()->load_IK_solver(__OR_robot, OpenRAVE::IKP_Transform6D);
      }

    } catch (Exception& e) {
      finalize();
      throw;
    }
  }
#endif //HAVE_OPENRAVE
}

void
KinovaOpenraveSingleThread::once()
{
#ifdef HAVE_OPENRAVE
  if(!__load_robot) {
    // robot was not loaded by this thread. So get them from openrave-environment now
    try {
      __OR_robot = openrave->get_active_robot();
      __OR_manip = __OR_robot->get_manipulator(); //TODO: use new(), copy constructor!

    } catch (Exception& e) {
      throw fawkes::Exception("Could not add robot '%s' to openrave environment. (Error: %s)", __cfg_OR_robot_file.c_str(), e.what_no_backtrace());
    }
  }

  while( !__robot ) {
    __robot = __OR_robot->get_robot_ptr();
    usleep(100);
  }
  while( !__manip ) {
    __manip = __robot->SetActiveManipulator(__manipname);
    usleep(100);
  }
#endif //HAVE_OPENRAVE
}

void
KinovaOpenraveSingleThread::finalize() {
  if(!__load_robot) {
    // avoid implicit deletes or anything the like
    __OR_robot = NULL;
    __OR_manip = NULL;
    __OR_env = NULL;
  } else {
    KinovaOpenraveBaseThread::finalize();
  }
}

void
KinovaOpenraveSingleThread::loop()
{
  if( __arm == NULL ) {
    usleep(30e3);
    return;
  }

  __planning_mutex->lock();
  __target_mutex->lock();
  if( !__target_queue->empty() ) {
    // get the target
    std::vector<float> target = __target_queue->front();
    __target_queue->pop_front();
    __target_mutex->unlock();

    // run planner
    _plan_path(target);
    __planning_mutex->unlock();

  } else {
    __target_mutex->unlock();
    __planning_mutex->unlock();
    usleep(30e3); // TODO: make this configurable
  }
}

void
KinovaOpenraveSingleThread::register_arm(jaco_arm_t *arm)
{
  __arm = arm;
}

void
KinovaOpenraveSingleThread::unregister_arms() {
  __arm = NULL;
}

void
KinovaOpenraveSingleThread::update_openrave()
{
  if( __arm == NULL || __robot == NULL || __manip == NULL )
    return;

#ifdef HAVE_OPENRAVE
  if( __planning_mutex->try_lock() ) {
    try {
      __joints.clear();
      __joints.push_back(__arm->iface->joints(0));
      __joints.push_back(__arm->iface->joints(1));
      __joints.push_back(__arm->iface->joints(2));
      __joints.push_back(__arm->iface->joints(3));
      __joints.push_back(__arm->iface->joints(4));
      __joints.push_back(__arm->iface->joints(5));

      // get target IK values in openrave format
      __OR_manip->set_angles_device(__joints);
      __OR_manip->get_angles(__joints);

      __robot->SetDOFValues(__joints, 1, __manip->GetArmIndices());
      __planning_mutex->unlock();

    } catch( openrave_exception &e) {
      __planning_mutex->unlock();
      throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
    }
  }
#endif
}

bool
KinovaOpenraveSingleThread::add_target(float x, float y, float z, float e1, float e2, float e3, bool plan)
{
  bool solvable = false;

#ifdef HAVE_OPENRAVE
  try {
    __OR_robot->get_robot_ptr()->SetActiveManipulator(__manip);
    __OR_robot->get_robot_ptr()->SetActiveDOFs(__manip->GetArmIndices());

    // update planner params; set correct DOF and stuff
    __OR_robot->get_planner_params()->SetRobotActiveJoints(__OR_robot->get_robot_ptr());
    __OR_robot->get_planner_params()->vgoalconfig.resize(__OR_robot->get_robot_ptr()->GetActiveDOF());

    // get IK from openrave
    solvable = __OR_robot->set_target_euler(EULER_ZXZ, x, y, z, e1, e2, e3);

    if( solvable ) {
      logger->log_debug(name(), "IK successful!");

      // get target IK valoues
      vector<float> joints;
      __OR_robot->get_target().manip->get_angles(joints);
      //need next lines, as "target" only stores a OpenRaveManipulator* , so it stores values in OR only!!
      __OR_manip->set_angles(joints);
      __OR_manip->get_angles_device(joints);

      if( plan ) {
        // add this to the target queue for planning
         logger->log_debug(name(), "Adding to target_queue for later planning");
        __target_mutex->lock();
        __target_queue->push_back(joints);
        __target_mutex->unlock();

       } else {
         // don't plan, consider this the final configuration, i.e. a trajectory with only 1 point
         logger->log_debug(name(), "Skip planning, add this to trajec_queue");
         vector< vector<float> > *trajec = new vector< vector<float> >();
         trajec->push_back(joints);
         __trajec_mutex->lock();
         __trajec_queue->push_back(trajec);
         __trajec_mutex->unlock();
       }

    } else {
      logger->log_warn(name(), "No IK solution found for target.");
      return solvable;
    }


  } catch( openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
#endif

  return solvable;
}

bool
KinovaOpenraveSingleThread::set_target(float x, float y, float z, float e1, float e2, float e3, bool plan)
{
  __target_mutex->lock();
  __target_queue->clear();
  __target_mutex->unlock();
  __trajec_mutex->lock();
  __trajec_queue->clear();
  __trajec_mutex->unlock();
  return add_target(x, y, z, e1, e2, e3, plan);
}

void
KinovaOpenraveSingleThread::_plan_path(std::vector<float> &target)
{

  // Set active manipulator
  __robot->SetActiveManipulator(__manip);
  __robot->SetActiveDOFs(__manip->GetArmIndices());

  // Set target point for planner (has already passed IK check previously!)
  __OR_manip->set_angles_device(target);
  __OR_manip->get_angles(target);
  //logger->log_debug(name(), "setting target %f %f %f %f %f %f",
  //                  target.at(0), target.at(1), target.at(2), target.at(3), target.at(4), target.at(5));
  __OR_robot->set_target_angles(target);

  // Set starting point for planner, convert encoder values to angles if necessary
  std::vector<float> joints;
  __arm->arm->get_joints(joints);
  //logger->log_debug(name(), "setting start %f %f %f %f %f %f",
  //                  joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5));
  __OR_manip->set_angles_device(joints);

  // Set planning parameters (none yet)
  __OR_robot->set_target_plannerparams("");

  // Run planner
  float sampling = 0.01f; //maybe catch from config? or "learning" depending on performance?
  try {
    openrave->run_planner(__OR_robot, sampling);
  } catch (fawkes::Exception &e) {
    logger->log_warn(name(), "Planning failed: %s", e.what_no_backtrace());
    return;
  }

  // add trajectory to queue
  //logger->log_debug(name(), "plan successful, adding to queue");
  __trajec_mutex->lock();
  std::vector< std::vector<float> > *trajec = __OR_robot->get_trajectory_device();
  __trajec_queue->push_back(trajec);
  __trajec_mutex->unlock();
}
