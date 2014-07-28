
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

#include <libkindrv/kindrv.h>

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
using namespace KinDrv;

/** @class KinovaOpenraveSingleThread "openrave_single_thread.h"
 * Jaco Arm thread for single-arm setup, integrating OpenRAVE
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaOpenraveSingleThread::KinovaOpenraveSingleThread()
  : KinovaOpenraveBaseThread("KinovaOpenraveSingleThread")
{
  __arm = NULL;
#ifdef HAVE_OPENRAVE
#endif
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

std::vector<float>
KinovaOpenraveSingleThread::set_target(float x, float y, float z, float e1, float e2, float e3)
{
  std::vector<float> v;

  try {
    std::vector<dReal> joints;
    // get IK from openrave
    bool success = __OR_robot->set_target_euler(EULER_ZXZ, x, y, z, e1, e2, e3);

    if( !success ) {
      logger->log_warn(name(), "Initiating goto failed, no IK solution found");
      return v;
    }
    logger->log_debug(name(), "IK successful!");

    // get target IK valoues
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
KinovaOpenraveSingleThread::loop()
{
  if( __arm == NULL )
    return;

#ifdef HAVE_OPENRAVE
//*
  try {
    std::vector<dReal> joints;
    joints.push_back(__arm->iface->joints(0));
    joints.push_back(__arm->iface->joints(1));
    joints.push_back(__arm->iface->joints(2));
    joints.push_back(__arm->iface->joints(3));
    joints.push_back(__arm->iface->joints(4));
    joints.push_back(__arm->iface->joints(5));

    // get target IK values in openrave format
    __OR_manip->set_angles_device(joints);
    __OR_manip->get_angles(joints);

    //EnvironmentMutex::scoped_lock lock(__OR_env->get_env_ptr()->GetMutex());
    __OR_robot->get_robot_ptr()->SetActiveDOFValues(joints);
    //usleep(2000);
  } catch( openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
//*/
#endif
}


void
KinovaOpenraveSingleThread::_load_robot()
{
#ifdef HAVE_OPENRAVE
  __cfg_OR_robot_file    = config->get_string("/hardware/jaco/openrave/robot_file");

  try {
    __OR_robot = openrave->add_robot(__cfg_OR_robot_file, false);

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
    // TODO: not just simple throw....
    throw;
  }
#endif
}
