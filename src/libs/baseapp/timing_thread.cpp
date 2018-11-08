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
  clock_ = Clock::instance();
  loop_start_ = new Time(clock_);
  loop_end_ = new Time(clock_);

  syncpoint_loop_start_ = syncpoint_manager->get_syncpoint(name(),
      "/preloop/start");
  syncpoint_loop_end_ = syncpoint_manager->get_syncpoint(name(),
      "/postloop/end");

  try {
    desired_loop_time_usec_ =
        config->get_uint("/fawkes/mainapp/desired_loop_time");
  } catch (Exception &e) {
    desired_loop_time_usec_ = 0;
    logger->log_info(name(), "Desired loop time not set, assuming 0");
  }
  desired_loop_time_sec_  = (float)desired_loop_time_usec_ / 1000000.f;

  try {
    min_loop_time_usec_ =
        config->get_uint("/fawkes/mainapp/min_loop_time");
  } catch (Exception &e) {
    min_loop_time_usec_ = 0;
    logger->log_info(name(), "Minimal loop time not set, assuming 0");
  }
  min_loop_time_sec_  = (float)min_loop_time_usec_ / 1000000.f;

  try {
    enable_looptime_warnings_ =
      config->get_bool("/fawkes/mainapp/enable_looptime_warnings");
    if(!enable_looptime_warnings_) {
      logger->log_debug(name(), "loop time warnings are disabled");
    }
  } catch(Exception &e) {
    enable_looptime_warnings_ = true;
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
  syncpoint_loop_start_->wait(name());
  loop_start_->stamp_systime();

  syncpoint_loop_end_->wait(name());
  loop_end_->stamp_systime();
  float loop_time = *loop_end_ - loop_start_;

  if (loop_time < min_loop_time_usec_) {
    logger->log_warn(name(), "Minimal loop time not reached, extending loop");
    usleep(min_loop_time_usec_ - loop_time);
    loop_end_->stamp_systime();
    loop_time = *loop_end_ - loop_start_;
  }

  if (desired_loop_time_sec_ > 0) {
    if(enable_looptime_warnings_) {
      // give some extra 10% to eliminate frequent false warnings due to regular
      // time jitter (TimeWait might not be all that precise)
      if (loop_time > 1.1 * desired_loop_time_sec_) {
        logger->log_warn(name(), "Loop time exceeded, "
            "desired: %f sec (%u usec),  actual: %f sec",
            desired_loop_time_sec_, desired_loop_time_usec_,
            loop_time);
      } else {
        logger->log_warn(name(), "Desired loop time achieved, "
            "desired: %f sec (%u usec),  actual: %f sec",
            desired_loop_time_sec_, desired_loop_time_usec_,
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
  syncpoint_manager->release_syncpoint(name(), syncpoint_loop_start_);
  syncpoint_manager->release_syncpoint(name(), syncpoint_loop_end_);
  delete loop_start_;
  delete loop_end_;
}

} // end namespace fawkes
