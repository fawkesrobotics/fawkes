
/***************************************************************************
 *  logreplay_bt_thread.h - BB Log Replay Blocked Timing Thread
 *
 *  Created: Sun Feb 21 23:50:08 2010
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

#ifndef __PLUGINS_BBLOGGER_LOGREPLAY_BT_THREAD_H_
#define __PLUGINS_BBLOGGER_LOGREPLAY_BT_THREAD_H_

#include "logreplay_thread.h"

#include <aspect/blocked_timing.h>

class BBLogReplayBlockedTimingThread
: public BBLogReplayThread,
  public fawkes::BlockedTimingAspect
{
 public:
  BBLogReplayBlockedTimingThread(fawkes::BlockedTimingAspect::WakeupHook hook,
				 const char *logfile_name,
				 const char *logdir, 
				 const char *scenario,
				 float grace_period,
				 bool loop_replay,
				 bool non_blocking);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

};


#endif
