
/***************************************************************************
 *  timesync_thread.cpp - Thread to synchronize loops, PRE_LOOP hook
 *
 *  Created: Tue Sep 30 14:29:59 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include "timesync_thread.h"

using namespace fawkes;

/** @class PlayerTimeSyncThread "playerc_thread.h"
 * Synchronization thread Time Sync
 * This thread is called in the PRE_LOOP hook to synchronize a Player
 * simulation loop and a Fawkes loop. Additionally, it provides a simulation
 * time source based on time received during the synchronization.
 * @author Tim Niemueller
 *
 * @todo needs real integration, currently only place holder
 */


/** Constructor. */
PlayerTimeSyncThread::PlayerTimeSyncThread()
  : Thread("PlayerTimeSyncThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP)
    //TimeSourceAspect(&__simts)
{
}


void
PlayerTimeSyncThread::init()
{
}


void
PlayerTimeSyncThread::finalize()
{
}


void
PlayerTimeSyncThread::loop()
{
}
