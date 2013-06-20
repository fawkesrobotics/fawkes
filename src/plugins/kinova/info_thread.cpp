
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
#include "kinova_api.h"

#include <interfaces/JacoInterface.h>

using namespace fawkes;

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
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
  __arm = NULL;
  __if_jaco = NULL;
}


/** Destructor. */
KinovaInfoThread::~KinovaInfoThread()
{
}

void
KinovaInfoThread::init()
{
  try {
    __if_jaco = blackboard->open_for_writing<JacoInterface>("JacoArm");
  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Could not open JacoInterface interface for writing. Er:%s", e.what());
  }
}

void
KinovaInfoThread::finalize()
{
  try {
    blackboard->close(__if_jaco);
  } catch(fawkes::Exception& e) {
    logger->log_warn(name(), "Could not close JacoInterface interface. Er:%s", e.what());
  }
}

void
KinovaInfoThread::register_arm(JacoArm *arm) {
  __arm = arm;
}

void
KinovaInfoThread::unregister_arm() {
  __arm = NULL;
}

void
KinovaInfoThread::loop()
{
  if(__arm != NULL) {
    __if_jaco->set_connected(true);

    try {
      __cpos = __arm->get_cart_pos();
      __if_jaco->set_x(-__cpos.Position[1]);
      __if_jaco->set_y( __cpos.Position[0]);
      __if_jaco->set_z( __cpos.Position[2]);

      __if_jaco->set_euler1(__cpos.Rotation[0]);
      __if_jaco->set_euler2(__cpos.Rotation[1]);
      __if_jaco->set_euler3(__cpos.Rotation[2]);

      __if_jaco->set_finger1(__cpos.FingerPosition[0]);
      __if_jaco->set_finger2(__cpos.FingerPosition[1]);
      __if_jaco->set_finger3(__cpos.FingerPosition[2]);

      __apos = __arm->get_ang_pos();
      __if_jaco->set_joints(__apos.Joints);

    } catch(fawkes::Exception &e) {
      logger->log_warn(name(), "Could not get position and joint values. Er: %s", e.what());
    }

  } else {
    __if_jaco->set_connected(false);
  }

  __if_jaco->write();
}