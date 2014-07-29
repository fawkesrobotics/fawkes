
/***************************************************************************
 *  openrave_dual_thread.cpp - Kinova plugin OpenRAVE Thread for dual-arm setup
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

#include "openrave_dual_thread.h"
#include "types.h"

#include <libkindrv/kindrv.h>

#include <interfaces/JacoInterface.h>

#include <cmath>
#include <stdio.h>
#include <cstring>

#ifdef HAVE_OPENRAVE
 #include <openrave/openrave.h>
 #include <plugins/openrave/environment.h>
 #include <plugins/openrave/robot.h>
 #include <plugins/openrave/manipulator.h>
 #include <plugins/openrave/manipulators/kinova_jaco.h>
 using namespace OpenRAVE;
#endif

// manipulator names in OpenRAVE
#define ARM_L "arm_left"
#define ARM_R "arm_right"

using namespace KinDrv;
using namespace fawkes;

/** @class KinovaOpenraveDualThread "openrave_dual_thread.h"
 * Jaco Arm thread for dual-arm setup, integrating OpenRAVE
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaOpenraveDualThread::KinovaOpenraveDualThread()
  : KinovaOpenraveBaseThread("KinovaOpenraveDualThread")
{
}

void
KinovaOpenraveDualThread::register_arm(fawkes::jaco_arm_t *arm)
{
  if( strcmp(arm->name, __cfg_left_arm_name.c_str()) == 0 ) {
    __arms.left = arm;
    logger->log_debug(name(), "Set arm '%s' as left arm.", arm->name);
  } else {
    __arms.right = arm;
    logger->log_debug(name(), "Set arm '%s' as right arm.", arm->name);
  }
}

void
KinovaOpenraveDualThread::unregister_arms()
{
  __arms.left = NULL;
  __arms.right = NULL;
}

std::vector<float>
KinovaOpenraveDualThread::set_target(float x, float y, float z, float e1, float e2, float e3, jaco_arm_t *arm)
{
  std::vector<float> v;

  try {
    if( arm == __arms.left ) {
      __OR_robot->get_robot_ptr()->SetActiveManipulator(ARM_L);
      __OR_robot->get_robot_ptr()->SetActiveDOFs(__manips.left->GetArmIndices());
    } else {
      __OR_robot->get_robot_ptr()->SetActiveManipulator(ARM_R);
      __OR_robot->get_robot_ptr()->SetActiveDOFs(__manips.right->GetArmIndices());
    }
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

    // get target IK values
    std::vector<dReal> joints;
    __OR_robot->get_target().manip->get_angles(joints);
    //need next lines, as "target" only stores a OpenRaveManipulator* , so it stores values in OR only!!
    __OR_manip->set_angles(joints);
    __OR_manip->get_angles_device(v);

  } catch( openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }

  return v;
}

void
KinovaOpenraveDualThread::loop()
{
  if( (__arms.left == NULL) || (__arms.right == NULL))
    return;

#ifdef HAVE_OPENRAVE
//*
  try {
    __manips.joints_l.clear();
    __manips.joints_l.push_back(__arms.left->iface->joints(0));
    __manips.joints_l.push_back(__arms.left->iface->joints(1));
    __manips.joints_l.push_back(__arms.left->iface->joints(2));
    __manips.joints_l.push_back(__arms.left->iface->joints(3));
    __manips.joints_l.push_back(__arms.left->iface->joints(4));
    __manips.joints_l.push_back(__arms.left->iface->joints(5));
    __manips.joints_r.clear();
    __manips.joints_r.push_back(__arms.right->iface->joints(0));
    __manips.joints_r.push_back(__arms.right->iface->joints(1));
    __manips.joints_r.push_back(__arms.right->iface->joints(2));
    __manips.joints_r.push_back(__arms.right->iface->joints(3));
    __manips.joints_r.push_back(__arms.right->iface->joints(4));
    __manips.joints_r.push_back(__arms.right->iface->joints(5));

    // get target IK values in openrave format
    __OR_manip->set_angles_device(__manips.joints_l);
    __OR_manip->get_angles(__manips.joints_l);
    __OR_manip->set_angles_device(__manips.joints_r);
    __OR_manip->get_angles(__manips.joints_r);

    //EnvironmentMutex::scoped_lock lock(__OR_env->get_env_ptr()->GetMutex());
    __OR_robot->get_robot_ptr()->SetDOFValues(__manips.joints_l, 1, __manips.left->GetArmIndices());
    __OR_robot->get_robot_ptr()->SetDOFValues(__manips.joints_r, 1, __manips.right->GetArmIndices());
    //usleep(2000);

  } catch( openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
//*/
#endif
}


void
KinovaOpenraveDualThread::_init()
{
  __cfg_left_arm_name = config->get_string("/hardware/jaco/dual_arm/left/name");
  __cfg_left_arm_name.resize(19, ' ');
}

void
KinovaOpenraveDualThread::_load_robot()
{
#ifdef HAVE_OPENRAVE
  __cfg_OR_robot_file  = config->get_string("/hardware/jaco/openrave/robot_dual_file");

  try {
    __OR_env = openrave->get_environment();
    __OR_env->enable_debug();

    //__OR_robot = openrave->add_robot(__cfg_OR_robot_file, false);
    // manually add robot; the automatic needs to be altered
    __OR_robot = new OpenRaveRobot(logger);
    __OR_robot->load(__cfg_OR_robot_file, __OR_env);
    __OR_env->add_robot(__OR_robot);
    __OR_robot->set_ready();
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

    __OR_robot->get_robot_ptr()->SetActiveManipulator(ARM_R);
    __manips.right = __OR_robot->get_robot_ptr()->GetActiveManipulator();
    if( __cfg_OR_auto_load_ik ) {
      logger->log_debug(name(), "load IK for right arm");
      __OR_env->load_IK_solver(__OR_robot, OpenRAVE::IKP_Transform6D);
    }

    __OR_robot->get_robot_ptr()->SetActiveManipulator(ARM_L);
    __manips.left = __OR_robot->get_robot_ptr()->GetActiveManipulator();
    if( __cfg_OR_auto_load_ik ) {
      logger->log_debug(name(), "load IK for left arm");
      __OR_env->load_IK_solver(__OR_robot, OpenRAVE::IKP_Transform6D);
    }

  } catch (Exception& e) {
    finalize();
    throw;
  }

#endif
}
