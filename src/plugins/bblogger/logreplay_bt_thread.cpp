
/***************************************************************************
 *  logreplay_bt_thread.cpp - BB Log Replay Blocked Timing Thread
 *
 *  Created: Mon Feb 22 00:09:58 2010
 *  Copyright  2010  Tim Niemueller [www.niemueller.de]
 *             2010  Masrur Doostdar <doostdar@kbsg.rwth-aachen.de>
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

#include "logreplay_bt_thread.h"

using namespace fawkes;

/** @class BBLogReplayBlockedTimingThread "logreplay_bt_thread.h"
 * BlackBoard log replay blocked timing thread.
 * This thread basically does the same task as the BBLogReplayThread, with
 * the difference that this thread operates in blocked timing mode and
 * integrates into the Fawkes main loop. It will replay one data set per
 * loop and will block the loop until the data is ready. This is most useful
 * to replay sensor logs of sensors, which are integrated into the main loop
 * and thus influence timing also on the real data system.
 * @author Masrur Doostdar
 * @author Tim Niemueller
 */

/** Constructor.
 * @param hook main loop hook to register for
 * @param logfile_name filename of the log to be replayed
 * @param logdir directory containing the logfile
 * @param scenario ID of the log scenario
 * @param grace_period time in seconds that desired offset and loop offset may
 * diverge to still write the new data
 * @param loop_replay specifies if the replay should be looped
 * @param non_blocking do not block the main loop if not enough time has elapsed
 * to replay new data but just wait for the next cycle. This is ignored in
 * continuous thread mode as it could cause busy waiting.
 */
BBLogReplayBlockedTimingThread::BBLogReplayBlockedTimingThread(
				   BlockedTimingAspect::WakeupHook hook,
				   const char *logfile_name,
				   const char *logdir,
				   const char *scenario,
				   float grace_period,
				   bool loop_replay,
				   bool non_blocking)
  : BBLogReplayThread(logfile_name, logdir, scenario,
		      grace_period, loop_replay, non_blocking,
		      "BBLogReplayBTThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(hook)
{
  set_name("BBLogReplayBTThread(%s)", logfile_name);
  set_prepfin_conc_loop(false);
}
