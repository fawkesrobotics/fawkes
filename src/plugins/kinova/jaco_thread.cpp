
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

using namespace fawkes;

/** @class JacoThread "jaco_thread.h"
 * Jaco Arm control thread.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
JacoThread::JacoThread()
  : Thread("JacoThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC)
{
  __arm = NULL;
}


/** Destructor. */
JacoThread::~JacoThread()
{
}

void
JacoThread::init()
{
  __arm = new JacoArm();

  //__arm->_get_device_handle();
}

void
JacoThread::finalize()
{
  delete __arm;
}

void
JacoThread::loop()
{
}