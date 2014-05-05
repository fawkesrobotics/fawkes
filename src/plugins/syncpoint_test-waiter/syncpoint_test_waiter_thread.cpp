  /***************************************************************************
 *  syncpoint_test_waiter_thread.cpp - SyncPoint Test Waiter Plugin
 *
 *   Created on Wed Jan 08 17:12:13 2014
 *   Copyright  2014  Till Hofmann
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


#include "syncpoint_test_waiter_thread.h"

using namespace fawkes;

/** @class SyncPointTestWaiterThread "syncpoint_test_waiter_thread.h"
 * Thread to test sync points
 * @author Till Hofmann
 */

/** Constructor. */
SyncPointTestWaiterThread::SyncPointTestWaiterThread()
  : Thread("SyncPointTestWaiterThread", Thread::OPMODE_CONTINUOUS)
   // BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

void
SyncPointTestWaiterThread::init()
{
  syncpoint_ = syncpoint_manager->get_syncpoint(name(), "/test/1");
  loopcount_ = 0;
}

void
SyncPointTestWaiterThread::finalize()
{
  syncpoint_manager->release_syncpoint(name(), syncpoint_);
}

void
SyncPointTestWaiterThread::loop()
{
  ++loopcount_;

  if (loopcount_ == 100) {
    // release syncpoint
    logger->log_debug(name(), "releasing syncpoint %s", syncpoint_->get_identifier());
    syncpoint_manager->release_syncpoint(name(), syncpoint_);
    return;
  }

  if (loopcount_ == 101) {
    // get it back
    syncpoint_manager->get_syncpoint(name(), "/test/1");
    logger->log_debug(name(), "regained syncpoint %s", syncpoint_->get_identifier());
  }

  if (loopcount_ == 150) {
    // release first sync point get a new one
    logger->log_debug(name(), "releasing syncpoint %s", syncpoint_->get_identifier());
    syncpoint_manager->release_syncpoint(name(), syncpoint_);
    syncpoint_ = syncpoint_manager->get_syncpoint(name(), "/test/2");
    logger->log_debug(name(), "regained syncpoint %s", syncpoint_->get_identifier());
  }

  // this should put this thread on hold for many iterations
  // that's not good, but it's only a test

  logger->log_info(name(), "[%u] waiting for syncpoint %s", loopcount_, syncpoint_->get_identifier());
  syncpoint_->wait(name());
  logger->log_info(name(), "[%u] got syncpoint %s", loopcount_, syncpoint_->get_identifier());

}
