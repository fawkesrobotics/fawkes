/***************************************************************************
 *  test_thread.cpp - Thread to test SyncPoints
 *
 *  Created: Thu Mar 05 15:15:42 2015
 *  Copyright  2015  Till Hofmann
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

#include "test_thread.h"

#include <stdlib.h>
#include <unistd.h>

using namespace fawkes;

/** @class SyncPointTestThread "test_thread.h"
 * Thread to test SyncPoints.
 * This thread has the blocked timing aspect and belongs to a wakeup hook.
 * It simply prints its name.
 * @author Till Hofmann
 */

/** Constructor.
 *  @param name the name of the thread
 *  @param hook the hook this thread belongs to
 */

SyncPointTestThread::SyncPointTestThread(const char * name, BlockedTimingAspect::WakeupHook hook)
  : Thread(name, Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(hook),
    hook_(hook)
{
}

void
SyncPointTestThread::init()
{
}

void
SyncPointTestThread::finalize()
{
}

void
SyncPointTestThread::loop()
{
  usleep(random() % 100000);
  logger->log_info("SyncPointTestThread", "In hook %s", BlockedTimingAspect::blocked_timing_hook_to_string(hook_));
}
