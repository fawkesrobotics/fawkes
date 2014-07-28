
/***************************************************************************
 *  info_thread.cpp - Kinova plugin Jaco information thread
 *
 *  Created: Thu Jun 13 19:14:20 2013
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

#include "info_thread.h"
#include "types.h"

#include <interfaces/JacoInterface.h>

#include <libkindrv/kindrv.h>

using namespace fawkes;
using namespace KinDrv;

/** @class KinovaInfoThread "info_thread.h"
 * Jaco Arm information thread.
 * This thread basically provides all informationen to interfaces.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaInfoThread::KinovaInfoThread()
  : Thread("KinovaInfoThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
  __arms = NULL;
}


/** Destructor. */
KinovaInfoThread::~KinovaInfoThread()
{
}

void
KinovaInfoThread::init()
{
  __arms = new std::list<jaco_arm_t*>();
}

void
KinovaInfoThread::finalize()
{
  delete(__arms);
}

void
KinovaInfoThread::register_arm(jaco_arm_t *arm) {
  __arms->push_back(arm);
}

void
KinovaInfoThread::unregister_arms() {
  for(__arm=__arms->begin(); __arm!=__arms->end(); ++__arm)
    (*__arm) = NULL;
}

void
KinovaInfoThread::loop()
{
  if( __arms->size() == 0 )
    return;

  for(__arm=__arms->begin(); __arm!=__arms->end(); ++__arm) {

    (*__arm)->iface->set_connected(true);

    try {
      __cpos = (*__arm)->arm->get_cart_pos();
      (*__arm)->iface->set_x(-__cpos.position[1]);
      (*__arm)->iface->set_y( __cpos.position[0]);
      (*__arm)->iface->set_z( __cpos.position[2]);

      (*__arm)->iface->set_euler1(__cpos.rotation[0]);
      (*__arm)->iface->set_euler2(__cpos.rotation[1]);
      (*__arm)->iface->set_euler3(__cpos.rotation[2]);

      (*__arm)->iface->set_finger1(__cpos.finger_position[0]);
      (*__arm)->iface->set_finger2(__cpos.finger_position[1]);
      (*__arm)->iface->set_finger3(__cpos.finger_position[2]);

      __apos = (*__arm)->arm->get_ang_pos();
      (*__arm)->iface->set_joints(__apos.joints);

    } catch(fawkes::Exception &e) {
      logger->log_warn(name(), "Could not get position and joint values. Er: %s", e.what());
    }
  }
}
