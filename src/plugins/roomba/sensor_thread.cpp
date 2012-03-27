
/***************************************************************************
 *  sensor_thread.cpp - Roomba plugin sensor thread
 *
 *  Created: Mon Jan 03 00:05:32 2011
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include "sensor_thread.h"
#include "thread_roomba_500.h"

using namespace fawkes;

/** @class RoombaSensorThread "sensor_thread.h"
 * Roomba sensor hook integration thread.
 * This thread integrates into the Fawkes main loop at the SENSOR hook and
 * triggers the Roomba thread to write new sensor data.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param roomba500_thread Roomba 500 thread to trigger to write to blackboard.
 */
RoombaSensorThread::RoombaSensorThread(Roomba500Thread *roomba500_thread)
  : Thread("RoombaSensorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
  __roomba500_thread = roomba500_thread;
}



void
RoombaSensorThread::loop()
{
  try {
    __roomba500_thread->write_blackboard();
  } catch (Exception &e) {
    logger->log_warn(name(), "%s failed to write to BB, exception follows.",
		     __roomba500_thread->name());
    logger->log_warn(name(), e);
  }
}
