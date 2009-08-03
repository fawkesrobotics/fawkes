
/***************************************************************************
 *  postsync_thread.cpp - Thread to synchronize loops, POST_LOOP hook
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

#include "postsync_thread.h"

using namespace fawkes;

/** @class PlayerPostSyncThread "playerc_thread.h"
 * Synchronization thread Post Sync
 * This thread is called in the POST_LOOP hook to synchronize a Player
 * simulation loop and a Fawkes loop.
 * @author Tim Niemueller
 *
 * @todo needs real integration, currently only place holder
 */


/** Constructor. */
PlayerPostSyncThread::PlayerPostSyncThread()
  : Thread("PlayerPostSyncThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP)
{
}


void
PlayerPostSyncThread::init()
{
}


void
PlayerPostSyncThread::finalize()
{
}


void
PlayerPostSyncThread::loop()
{
}
