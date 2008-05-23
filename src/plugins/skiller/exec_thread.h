
/***************************************************************************
 *  exec_thread.h - Fawkes Skiller: Execution Thread
 *
 *  Created: Mon Feb 18 10:28:38 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __PLUGINS_SKILLER_EXEC_THREAD_H_
#define __PLUGINS_SKILLER_EXEC_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <utils/system/fam.h>

extern "C" {
#include <lua.h>
}

#include <string>
#include <cstdlib>

class SkillerLiaisonThread;
namespace fawkes {
  class ComponentLogger;
  class Mutex;
}

class SkillerExecutionThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::FamListener
{
 public:
  SkillerExecutionThread(fawkes::Barrier *liaison_exec_barrier,
			 SkillerLiaisonThread *slt);
  virtual ~SkillerExecutionThread();

  virtual void init();
  virtual void once();
  virtual void loop();
  virtual void finalize();

  void skiller_reader_removed(unsigned int instance_serial);

  virtual void fam_event(const char *filename, unsigned int mask);

 private: /* methods */
  void init_lua();
  void start_lua();
  void restart_lua();
  void publish_skill_status(std::string &curss);

 private: /* members */
  fawkes::Barrier *__liaison_exec_barrier;
  SkillerLiaisonThread *__slt;
  fawkes::ComponentLogger *__clog;

  lua_State *__L;
  int __err;
  std::string __errmsg;
  fawkes::Mutex *__lua_mutex;

  bool        __continuous_run;
  bool        __continuous_rst;

  // config values
  std::string __cfg_skillspace;
  bool        __cfg_watch_files;

  fawkes::FileAlterationMonitor *__fam;
};

#endif
