
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
#include "arm.h"

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
  __arms.left = NULL;
  __arms.right = NULL;
}

void
KinovaOpenraveDualThread::_init()
{
  __cfg_left_arm_name = config->get_string("/hardware/jaco/dual_arm/left/name");
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
    openrave->set_active_robot(__OR_robot);
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


void
KinovaOpenraveDualThread::register_arm(fawkes::jaco_arm_t *arm)
{
  if( __cfg_left_arm_name.compare(arm->arm->get_name()) == 0 ) {
    __arms.left = arm;
    logger->log_debug(name(), "Set arm '%s' as left arm.", arm->arm->get_name().c_str());
  } else {
    __arms.right = arm;
    logger->log_debug(name(), "Set arm '%s' as right arm.", arm->arm->get_name().c_str());
  }
}

void
KinovaOpenraveDualThread::unregister_arms()
{
  __arms.left = NULL;
  __arms.right = NULL;
}

std::vector<float>
KinovaOpenraveDualThread::set_target(float x, float y, float z, float e1, float e2, float e3)
{
  return std::vector<float>(0);
  // no symmetric planning implemented yet
}

void
KinovaOpenraveDualThread::update_openrave()
{
  // do nothing, this thread is only for plannning!
}
