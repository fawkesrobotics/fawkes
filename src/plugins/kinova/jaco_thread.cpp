
/***************************************************************************
 *  jaco_thread.cpp - Kinova plugin Jaco Thread
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

#include "jaco_thread.h"
#include "kinova_api.h"

#include <interfaces/JacoInterface.h>

using namespace fawkes;

/** @class KinovaJacoThread "jaco_thread.h"
 * Jaco Arm control thread.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaJacoThread::KinovaJacoThread(KinovaInfoThread *info_thread)
  : Thread("KinovaJacoThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC)
{
  __arm = NULL;
  __if_jaco = NULL;
  __info_thread = info_thread;
}


/** Destructor. */
KinovaJacoThread::~KinovaJacoThread()
{
}

void
KinovaJacoThread::init()
{
  try {
    // create new JacoArm object (connects to arm via libusb)
    __arm = new JacoArm();

    // register arm in KinovaInfoThread
    __info_thread->register_arm(__arm);

  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Could not connect to JacoArm. Ex:%s", e.what());
  }


  try {
    // open interface for writing
    __if_jaco = blackboard->open_for_writing<JacoInterface>("JacoArm");

    // set interface in other threads
    __info_thread->set_interface(__if_jaco);
  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Could not open JacoInterface interface for writing. Er:%s", e.what());
  }
}

void
KinovaJacoThread::finalize()
{
  try {
    blackboard->close(__if_jaco);
  } catch(fawkes::Exception& e) {
    logger->log_warn(name(), "Could not close JacoInterface interface. Er:%s", e.what());
  }

  delete __arm;
}

void
KinovaJacoThread::loop()
{
  __if_jaco->write();
}