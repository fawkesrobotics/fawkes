
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
#include <core/threading/mutex.h>

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
    __viewer_env.env = openrave->get_environment();
    __viewer_env.env->enable_debug();

    //__viewer_env.robot = openrave->add_robot(__cfg_OR_robot_file, false);
    // manually add robot; the automatic needs to be altered
    __viewer_env.robot = new OpenRaveRobot(logger);
    __viewer_env.robot->load(__cfg_OR_robot_file, __viewer_env.env);
    __viewer_env.env->add_robot(__viewer_env.robot);
    __viewer_env.robot->set_ready();
    openrave->set_active_robot(__viewer_env.robot);
  } catch (Exception& e) {
    throw fawkes::Exception("Could not add robot '%s' to openrave environment. (Error: %s)", __cfg_OR_robot_file.c_str(), e.what_no_backtrace());
  }

  try {
    __viewer_env.manip = new OpenRaveManipulatorKinovaJaco(6, 6);
    __viewer_env.manip->add_motor(0,0);
    __viewer_env.manip->add_motor(1,1);
    __viewer_env.manip->add_motor(2,2);
    __viewer_env.manip->add_motor(3,3);
    __viewer_env.manip->add_motor(4,4);
    __viewer_env.manip->add_motor(5,5);

    // Set manipulator and offsets.
    openrave->set_manipulator(__viewer_env.robot, __viewer_env.manip, 0.f, 0.f, 0.f);

    __viewer_env.robot->get_robot_ptr()->SetActiveManipulator(ARM_R);
    __manips.right = __viewer_env.robot->get_robot_ptr()->GetActiveManipulator();
    if( __cfg_OR_auto_load_ik ) {
      logger->log_debug(name(), "load IK for right arm");
      __viewer_env.env->load_IK_solver(__viewer_env.robot, OpenRAVE::IKP_Transform6D);
    }

    __viewer_env.robot->get_robot_ptr()->SetActiveManipulator(ARM_L);
    __manips.left = __viewer_env.robot->get_robot_ptr()->GetActiveManipulator();
    if( __cfg_OR_auto_load_ik ) {
      logger->log_debug(name(), "load IK for left arm");
      __viewer_env.env->load_IK_solver(__viewer_env.robot, OpenRAVE::IKP_Transform6D);
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

bool
KinovaOpenraveDualThread::add_target(float x, float y, float z, float e1, float e2, float e3, bool plan)
{
  // no IK-solving for coordinated bimanual movement implemented yet
  return false;
}

bool
KinovaOpenraveDualThread::set_target(float x, float y, float z, float e1, float e2, float e3, bool plan)
{
  __target_mutex->lock();
  __target_queue->clear();
  __target_mutex->unlock();
  return add_target(x, y, z, e1, e2, e3, plan);
}


void
KinovaOpenraveDualThread::update_openrave()
{
  // do nothing, this thread is only for plannning!
}

/** Plot the first target of the queue in the viewer_env */
void
KinovaOpenraveDualThread::plot_first()
{
}
