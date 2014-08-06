
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
#include "types.h"

#include <interfaces/JacoInterface.h>

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
      __OR_env = openrave->get_environment();
      __OR_env->enable_debug();
      __OR_robot = openrave->get_active_robot();
      __OR_manip = __OR_robot->get_manipulator(); //TODO: use new(), copy constructor!

    } catch (Exception& e) {
      throw fawkes::Exception("Could not add robot '%s' to openrave environment. (Error: %s)", __cfg_OR_robot_file.c_str(), e.what_no_backtrace());
    }
  }

  while( __robot == NULL ) {
    __robot = __OR_robot->get_robot_ptr();
    usleep(100);
  }
  while( __manip == NULL ) {
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

  } catch( openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
#endif
}

std::vector<float>
KinovaOpenraveSingleThread::set_target(float x, float y, float z, float e1, float e2, float e3)
{
  std::vector<float> v;

#ifdef HAVE_OPENRAVE
  try {
    __OR_robot->get_robot_ptr()->SetActiveManipulator(__manip);
    __OR_robot->get_robot_ptr()->SetActiveDOFs(__manip->GetArmIndices());

    // update planner params; set correct DOF and stuff
    __OR_robot->get_planner_params()->SetRobotActiveJoints(__OR_robot->get_robot_ptr());
    __OR_robot->get_planner_params()->vgoalconfig.resize(__OR_robot->get_robot_ptr()->GetActiveDOF());

    // get IK from openrave
    bool success = __OR_robot->set_target_euler(EULER_ZXZ, x, y, z, e1, e2, e3);

    if( !success ) {
      logger->log_warn(name(), "Initiating goto failed, no IK solution found");
      return v;
    }
    logger->log_debug(name(), "IK successful!");

    // get target IK valoues
    std::vector<dReal> joints;
    __OR_robot->get_target().manip->get_angles(joints);
    //need next lines, as "target" only stores a OpenRaveManipulator* , so it stores values in OR only!!
    __OR_manip->set_angles(joints);
    __OR_manip->get_angles_device(v);

  } catch( openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
#endif

  return v;
}
