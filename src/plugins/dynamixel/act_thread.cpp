
/***************************************************************************
 *  act_thread.cpp - Dynamixel plugin act thread
 *
 *  Created: Tue Mar 24 14:55:57 2015 (based on pantilt plugin)
 *  Copyright  2006-2015  Tim Niemueller [www.niemueller.de]
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

#include "act_thread.h"
#include "driver_thread.h"

using namespace fawkes;

/** @class DynamixelActThread "act_thread.h"
 * Robotis dynamixel servo act thread.
 * This thread integrates into the Fawkes main loop at the ACT_EXEC hook and
 * calls the driver thread to accept new actuation commands.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
DynamixelActThread::DynamixelActThread()
  : Thread("DynamixelActThread", Thread::OPMODE_CONTINUOUS),
    SyncPointAspect(SyncPoint::WAIT_FOR_ALL, "/sensors/acquire", "/act/exec/end")
{
}


void
DynamixelActThread::loop()
{
  for (auto &d : driver_threads_) {
    d->exec_act();
  }
}

/** Add a driver thread to wake in ACT hook.
 * @param drv_thread driver thread to add
 */
void
DynamixelActThread::add_driver_thread(DynamixelDriverThread *drv_thread)
{
  driver_threads_.push_back(drv_thread);
}
