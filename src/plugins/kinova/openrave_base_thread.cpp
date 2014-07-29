
/***************************************************************************
 *  openrave_thread.cpp - Kinova plugin OpenRAVE base Thread
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

#include "openrave_base_thread.h"

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

/** @class KinovaOpenraveBaseThread "openrave_base_thread.h"
 * Base Jaco Arm thread, integrating OpenRAVE
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaOpenraveBaseThread::KinovaOpenraveBaseThread(const char *name)
  : Thread(name, Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
#ifdef HAVE_OPENRAVE
  __OR_env   = NULL;
  __OR_robot = NULL;
  __OR_manip = NULL;

  __cfg_OR_auto_load_ik = false;
#endif
}


/** Destructor. */
KinovaOpenraveBaseThread::~KinovaOpenraveBaseThread()
{
#ifdef HAVE_OPENRAVE
  __OR_env   = NULL;
  __OR_robot = NULL;
  __OR_manip = NULL;
#endif
}

std::vector<float>
KinovaOpenraveBaseThread::set_target(float x, float y, float z, float e1, float e2, float e3, jaco_arm_t *arm)
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
KinovaOpenraveBaseThread::init()
{
#ifdef HAVE_OPENRAVE
  __cfg_OR_use_viewer    = config->get_bool("/hardware/jaco/openrave/use_viewer");
  __cfg_OR_auto_load_ik  = config->get_bool("/hardware/jaco/openrave/auto_load_ik");

  // perform other initialization stuff (for child classes, that do not want to overload "init()")
  _init();

  // load robot
  _load_robot();

  if( __cfg_OR_use_viewer )
    openrave->start_viewer();
#endif
}

void
KinovaOpenraveBaseThread::finalize()
{
  unregister_arms();

#ifdef HAVE_OPENRAVE
  delete(__OR_robot);
  __OR_robot = NULL;

  delete(__OR_manip);
  __OR_manip = NULL;

  __OR_env = NULL;
#endif
}
