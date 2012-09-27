
/***************************************************************************
 *  main_thread.h - Fawkes main thread
 *
 *  Created: Thu Nov  2 16:46:37 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __LIBS_BASEAPP_MAIN_THREAD_H_
#define __LIBS_BASEAPP_MAIN_THREAD_H_

#include <baseapp/thread_manager.h>
#include <core/threading/thread.h>
#include <aspect/mainloop/employer.h>
#include <aspect/blocked_timing.h>
#include <utils/system/signal.h>
#include <logging/multi.h>

#include <list>
#include <string>
#include <getopt.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif
class Configuration;
class Configuration;
class ConfigNetworkHandler;
class NetworkLogger;
class Clock;
class TimeWait;
class AspectManager;
class PluginManager;
class Time;
class PluginNetworkHandler;
class InterruptibleBarrier;
class Barrier;
class Mutex;
class ThreadManager;
class FawkesNetworkManager;

class FawkesMainThread
: public Thread,
  public MainLoopEmployer
{
 public:
  FawkesMainThread(Configuration *config,
		   MultiLogger *multi_logger,
		   ThreadManager *thread_manager,
		   PluginManager *plugin_manager,
		   const char *load_plugins,
                   const char *default_plugin = 0);
  virtual ~FawkesMainThread();

  virtual void once();
  virtual void loop();

  virtual void set_mainloop_thread(Thread *mainloop_thread);

  void full_start();

  MultiLogger *  logger() const;

  class Runner : public SignalHandler {
  public:
    Runner(FawkesMainThread *fmt, bool register_signals = true);
    ~Runner();
    void run();
    void handle_signal(int signum);
  private:
    FawkesMainThread *__fmt;
    Mutex            *__init_mutex;
    bool              __init_running;
    bool              __init_quit;
    bool              __sigint_running;
    bool              __register_signals;
  };

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void destruct();

  inline void safe_wake(BlockedTimingAspect::WakeupHook hook, unsigned int timeout_usec)
  {
    try {
      __thread_manager->wakeup_and_wait(hook, timeout_usec);
    } catch (Exception &e) {
      if (__enable_looptime_warnings) {
        //__multi_logger->log_error("FawkesMainThread",
        //                          "Error while processing hook %s, exception follows",
        //                          BlockedTimingAspect::blocked_timing_hook_to_string(hook));
        __multi_logger->log_error("FawkesMainThread", e);
      }

    }
  }


  Configuration        *__config;
  MultiLogger          *__multi_logger;
  NetworkLogger        *__network_logger;
  Clock                *__clock;
  TimeWait             *__time_wait;
  AspectManager        *__aspect_manager;

  Barrier              *__init_barrier;
  Thread               *__mainloop_thread;
  Mutex                *__mainloop_mutex;
  InterruptibleBarrier *__mainloop_barrier;

  char                 *__default_plugin;
  char                 *__load_plugins;

  ThreadManager        *__thread_manager;
  PluginManager        *__plugin_manager;
  Mutex                *__plugin_mutex;
  FawkesNetworkManager *__network_manager;

  std::list<std::string>        __recovered_threads;
  unsigned int                  __desired_loop_time_usec;
  float                         __desired_loop_time_sec;
  unsigned int                  __max_thread_time_usec;
  unsigned int                  __max_thread_time_nanosec;
  Time                         *__loop_start;
  Time                         *__loop_end;
  bool                          __enable_looptime_warnings;

};

} // end namespace fawkes

#endif
