  /***************************************************************************
 *  syncpoint_test_emitter_thread.cpp - SyncPoint Test Emitter Plugin
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


#include "syncpoint_test_emitter_thread.h"
#include <syncpoint/syncpoint_manager.h>

using namespace fawkes;

/** @class SyncPointTestEmitterThread "syncpoint_test_emitter_thread.h"
 * Thread to test sync points
 * @author Till Hofmann
 */

/** Constructor. */
SyncPointTestEmitterThread::SyncPointTestEmitterThread()
  : Thread("SyncPointTestEmitterThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

void
SyncPointTestEmitterThread::init()
{
  syncpoint_ = syncpoint_manager->get_syncpoint(name(), "/test/1");
}

void
SyncPointTestEmitterThread::finalize()
{
  syncpoint_manager->release_syncpoint(name(), syncpoint_);
}

void
SyncPointTestEmitterThread::loop()
{
  ++loopcount_;
  logger->log_debug(name(), "number of syncpoints: %u", syncpoint_manager->get_syncpoints().size());
  logger->log_info(name(), "[%u] emitting syncpoint %s", loopcount_, syncpoint_->get_identifier());
  syncpoint_->emit(name());

  if (loopcount_ == 300) {
    // release first syncpoint
    logger->log_debug(name(), "releasing syncpoint %s", syncpoint_->get_identifier());
    syncpoint_manager->release_syncpoint(name(), syncpoint_);
    // get a second syncpoint
    syncpoint_ = syncpoint_manager->get_syncpoint(name(), "/test/2");
    logger->log_debug(name(), "regained syncpoint %s", syncpoint_->get_identifier());
  }
}
