
/***************************************************************************
 *  openrave_thread.cpp - Kinova plugin OpenRAVE Thread
 *
 *  Created: Tue Jun 04 13:13:20 2013
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

#include "openrave_thread.h"
#include "kinova_api.h"

#include <interfaces/JacoInterface.h>

#include <plugins/openrave/environment.h>
#include <plugins/openrave/robot.h>
#include <plugins/openrave/manipulator.h>
#include <plugins/openrave/manipulators/kinova_jaco.h>

#include <cmath>


#include <stdio.h>
#include <cstring>

using namespace fawkes;
using namespace OpenRAVE;

/** @class JacoOpenraveThread "jaco_thread.h"
 * Jaco Arm control thread.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
JacoOpenraveThread::JacoOpenraveThread()
  : Thread("JacoOpenraveThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
  __arm = NULL;
  __OR_env   = NULL;
  __OR_robot = NULL;
  __OR_manip = NULL;
  cnt = 0;
}


/** Destructor. */
JacoOpenraveThread::~JacoOpenraveThread()
{
  __OR_env   = NULL;
  __OR_robot = NULL;
  __OR_manip = NULL;
}

void
JacoOpenraveThread::register_arm(JacoArm *arm)
{
  __arm = arm;
}

void
JacoOpenraveThread::init()
{
  logger->log_debug(name(), "init()");

  /*
  try {
    // open interface for reading
    __if_jaco = blackboard->open_for_reading<JacoInterface>("JacoArm");
    logger->log_debug(name(), "Interfaces openede for writing");

  } catch(fawkes::Exception &e) {
    logger->log_error(name(), "Could not open JacoInterface interface for writing. Er:%s", e.what());
  }
  //*/

  try {
    __OR_robot = openrave->add_robot("../fawkes/res/openrave/jaco.robot.xml", false);

    __OR_manip = new OpenRaveManipulatorKinovaJaco(6, 6);
    __OR_manip->add_motor(0,0);
    __OR_manip->add_motor(1,1);
    __OR_manip->add_motor(2,2);
    __OR_manip->add_motor(3,3);
    __OR_manip->add_motor(4,4);
    __OR_manip->add_motor(5,5);

    // Set manipulator and offsets.
    // offsetZ: katana.kinbody is 0.165 above ground; coordinate system of real katana has origin in intersection of j1 and j2 (i.e. start of link L2: 0.2015 on z-axis)
    // offsetX: katana.kinbody is setup 0.0725 on +x axis
    openrave->set_manipulator(__OR_robot, __OR_manip, 0.f, 0.f, 0.f);

    if( false ) {
      openrave->get_environment()->load_IK_solver(__OR_robot, OpenRAVE::IKP_Transform6D);
    }

  } catch (Exception& e) {
    // TODO: not just simple throw....
    throw;
  }

  if( true )
    openrave->start_viewer();

  logger->log_debug(name(), "init() done");
}

void
JacoOpenraveThread::finalize()
{
  /*
  try {
    blackboard->close(__if_jaco);
  } catch(fawkes::Exception& e) {
    logger->log_warn(name(), "Could not close JacoInterface interface. Er:%s", e.what());
  }
  //*/
  delete(__OR_robot);
  __OR_robot = NULL;

  delete(__OR_manip);
  __OR_manip = NULL;
}

void
JacoOpenraveThread::loop()
{
  if( __arm == NULL )
    return;

//*
  try {
    jaco_position_t pos = __arm->get_ang_pos();
    std::vector<dReal> joints;
    joints.push_back(pos.Joints[0]);
    joints.push_back(pos.Joints[1]);
    joints.push_back(pos.Joints[2]);
    joints.push_back(pos.Joints[3]);
    joints.push_back(pos.Joints[4]);
    joints.push_back(pos.Joints[5]);

    std::vector<dReal> v;
    __OR_manip->set_angles_device(joints);

    __OR_manip->get_angles(v);
    logger->log_debug(name(), "OR angles: %f  %f  %f  %f  %f  %f",
      v.at(0), v.at(1), v.at(2), v.at(3), v.at(4), v.at(5));

    //EnvironmentMutex::scoped_lock lock(__OR_env->get_env_ptr()->GetMutex());
    __OR_robot->get_robot_ptr()->SetActiveDOFValues(v);
    usleep(2000);
  } catch( openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
  //*/

}