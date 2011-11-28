
/***************************************************************************
 *  periodic_exec_thread.h - Fawkes LuaAgent: Periodic Execution Thread
 *
 *  Created: Thu Jan 01 11:12:13 2009
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LUAAGENT_PERIODIC_EXEC_THREAD_H_
#define __PLUGINS_LUAAGENT_PERIODIC_EXEC_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>
#ifdef HAVE_TF
#  include <aspect/tf.h>
#endif
#include <utils/system/fam.h>
#include <blackboard/interface_listener.h>

#include <string>
#include <cstdlib>

namespace fawkes {
  class ComponentLogger;
  class Mutex;
  class LuaContext;
  class LuaInterfaceImporter;
  class Interface;
  class SkillerInterface;
  class SkillerDebugInterface;
}

class LuaAgentPeriodicExecutionThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
#ifdef HAVE_TF
  public fawkes::TransformAspect,
#endif
  public fawkes::ClockAspect
{
 public:
  LuaAgentPeriodicExecutionThread();
  virtual ~LuaAgentPeriodicExecutionThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: /* methods */
  void init_failure_cleanup();
  void process_agdbg_messages();

 private: /* members */
  fawkes::ComponentLogger *__clog;

  // config values
  std::string __cfg_agent;
  bool        __cfg_watch_files;

  fawkes::SkillerInterface      *__skiller_if;
  fawkes::SkillerDebugInterface *__agdbg_if;

  fawkes::LuaContext  *__lua;
  fawkes::LuaInterfaceImporter  *__lua_ifi;
};

#endif
