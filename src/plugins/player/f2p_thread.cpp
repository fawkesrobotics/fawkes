
/***************************************************************************
 *  f2p_thread.cpp - Thread that calls mappers to sync Fawkes to Player
 *
 *  Created: Tue Sep 30 13:13:50 2008
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

#include "f2p_thread.h"
#include "playerc_thread.h"

using namespace fawkes;

/** @class PlayerF2PThread "playerc_thread.h"
 * Player Fawkes-To-Player Thread.
 * This thread is called in the ACT_EXEC hook to post any outstanding
 * Fawkes interface data to the Player proxies via mappers.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param client_thread PlayerClientThread to use for syncing Fawkes interfaces
 * to Player proxies.
 */
PlayerF2PThread::PlayerF2PThread(PlayerClientThread *client_thread)
  : Thread("PlayerF2PThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC)
{
  __client_thread = client_thread;
}


void
PlayerF2PThread::init()
{
}


void
PlayerF2PThread::finalize()
{
}


void
PlayerF2PThread::loop()
{
  __client_thread->sync_fawkes_to_player();
}
