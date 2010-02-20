
/***************************************************************************
 *  logreplay_thread.h - BB Log Replay Thread
 *
 *  Created: Mi Feb 17 01:53:00 2010
 *  Copyright  2010  Masrur Doostdar, Tim Niemueller [www.niemueller.de]
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
#include "file.h"
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
		    const char *logdir, const char *scenario);
  virtual ~BBLogReplayThread();

  virtual void init();
  virtual void finalize();
  virtual void once();


 private:
  void read_file_header(FILE *f, bblog_file_header *header);
  void sanity_check(FILE *f, bblog_file_header *header);
  void read_entry(FILE *f, bblog_file_header *header, bblog_entry_header *entryh,fawkes::Interface *iface, unsigned int index, bool do_seek = true);
  
  
 private:
  char               *__scenario;
  char               *__filename;
  char               *__logdir;
  char               *__logfile_name;
  FILE               *__f_data;

};


#endif
