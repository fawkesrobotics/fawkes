/***************************************************************************
 *  timing_thread.cpp - Timing thread to achieve a desired main loop time
 *
 *  Created: Thu Jul 23 14:45:42 2015
 *  Copyright  2015-2017  Till Hofmann
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


#include <baseapp/timing_thread.h>

#include <unistd.h>


#define CFG_PREFIX "/fawkes/mainapp/"

namespace fawkes {

/** @class FawkesTimingThread <baseapp/timing_thread.h>
 * Thread to control the main loop timing.
 * This thread uses the preloop and postloop SyncPoints to control the timing of
 * the main loop. It waits for the preloop SyncPoint at the beginning of the
 * main loop and emits the postloop SyncPoint at the end of the loop. If the
 * loop time is less than the desired loop time, the thread waits before
 * releasing the postloop SyncPoint. If the loop time is longer than the maximum
 * loop time, a warning is printed.
 */
FawkesTimingThread::FawkesTimingThread()
: Thread("FawkesTimingThread", Thread::OPMODE_CONTINUOUS)
//  ConfigurationChangeHandler(CFG_PREFIX)
{
}

/** Initialize.
 * Get the pre- and postloop SyncPoints and read all relevant config values.
 */
void
FawkesTimingThread::init()
{
  __clock = Clock::instance();
  __loop_start = new Time(__clock);
  __loop_end = new Time(__clock);

  __syncpoint_loop_start = syncpoint_manager->get_syncpoint(name(),
      "/preloop/start");
  __syncpoint_loop_end = syncpoint_manager->get_syncpoint(name(),
      "/postloop/end");

  try {
    __desired_loop_time_usec =
        config->get_uint("/fawkes/mainapp/desired_loop_time");
  } catch (Exception &e) {
    __desired_loop_time_usec = 0;
    logger->log_info(name(), "Desired loop time not set, assuming 0");
  }
  __desired_loop_time_sec  = (float)__desired_loop_time_usec / 1000000.f;

  try {
    __min_loop_time_usec =
        config->get_uint("/fawkes/mainapp/min_loop_time");
  } catch (Exception &e) {
    __min_loop_time_usec = 0;
    logger->log_info(name(), "Minimal loop time not set, assuming 0");
  }
  __min_loop_time_sec  = (float)__min_loop_time_usec / 1000000.f;

  try {
    __enable_looptime_warnings =
      config->get_bool("/fawkes/mainapp/enable_looptime_warnings");
    if(!__enable_looptime_warnings) {
      logger->log_debug(name(), "loop time warnings are disabled");
    }
  } catch(Exception &e) {
    __enable_looptime_warnings = true;
  }

}

/** Thread loop.
 * This loop runs parallel to the main loop. At the beginning of the main loop,
 * it waits for the preloop SyncPoint. At the end of the main loop, it emits the
 * postloop SyncPoint.
 */
void
FawkesTimingThread::loop()
{
  __syncpoint_loop_start->wait(name());
  __loop_start->stamp_systime();

  __syncpoint_loop_end->wait(name());
  __loop_end->stamp_systime();
  float loop_time = *__loop_end - __loop_start;

  if (loop_time < __min_loop_time_usec) {
    logger->log_warn(name(), "Minimal loop time not reached, extending loop");
    usleep(__min_loop_time_usec - loop_time);
    __loop_end->stamp_systime();
    loop_time = *__loop_end - __loop_start;
  }

  if (__desired_loop_time_sec > 0) {
    if(__enable_looptime_warnings) {
      // give some extra 10% to eliminate frequent false warnings due to regular
      // time jitter (TimeWait might not be all that precise)
      if (loop_time > 1.1 * __desired_loop_time_sec) {
        logger->log_warn(name(), "Loop time exceeded, "
            "desired: %f sec (%u usec),  actual: %f sec",
            __desired_loop_time_sec, __desired_loop_time_usec,
            loop_time);
      } else {
        logger->log_warn(name(), "Desired loop time achieved, "
            "desired: %f sec (%u usec),  actual: %f sec",
            __desired_loop_time_sec, __desired_loop_time_usec,
            loop_time);
      }
    }
  }
}

/** Finalize the thread.
 * Release all SyncPoints and do other cleanup.
 */
void
FawkesTimingThread::finalize()
{
  syncpoint_manager->release_syncpoint(name(), __syncpoint_loop_start);
  syncpoint_manager->release_syncpoint(name(), __syncpoint_loop_end);
  delete __loop_start;
  delete __loop_end;
}

} // end namespace fawkes
