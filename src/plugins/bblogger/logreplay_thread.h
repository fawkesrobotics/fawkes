
/***************************************************************************
 *  logreplay_thread.h - BB Log Replay Thread
 *
 *  Created: Wed Feb 17 01:53:00 2010
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

#ifndef __PLUGINS_BBLOGGER_LOGREPLAY_THREAD_H_
#define __PLUGINS_BBLOGGER_LOGREPLAY_THREAD_H_

#include "bblogfile.h"

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <core/utils/lock_queue.h>
 
#include <cstdio>

namespace fawkes {
  class BlackBoard;
  class Logger;
  class Time;
}

class BBLogReplayThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect
{
 public:
  BBLogReplayThread(const char *logfile_name,
		    const char *logdir, 
		    const char *scenario,
		    float grace_period,
		    bool loop_replay,
		    bool non_blocking = false,
		    const char *thread_name = "BBLogReplayThread",
		    fawkes::Thread::OpMode th_opmode = Thread::OPMODE_CONTINUOUS);
  virtual ~BBLogReplayThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();
  virtual void once();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  char               *__scenario;
  char               *__filename;
  char               *__logdir;
  char               *__logfile_name;
  float               __cfg_grace_period;
  bool                __cfg_non_blocking;
  bool                __cfg_loop_replay;

  BBLogFile          *__logfile;

  fawkes::Time        __last_offset;
  fawkes::Time        __offsetdiff;
  fawkes::Time        __loopdiff;
  fawkes::Time        __waittime;
  fawkes::Time        __last_loop;
  fawkes::Time        __now;
  fawkes::Interface  *__interface;
};


#endif
