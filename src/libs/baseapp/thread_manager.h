
/***************************************************************************
 *  thread_manager.h - Fawkes thread manager
 *
 *  Created: Thu Nov  3 19:08:23 2006 (on train to Cologne)
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_BASEAPP_THREAD_MANAGER_H_
#define __LIBS_BASEAPP_THREAD_MANAGER_H_

#include <core/threading/thread_list.h>
#include <core/threading/thread_collector.h>
#include <core/exception.h>
#include <aspect/blocked_timing.h>
#include <aspect/blocked_timing/executor.h>

#include <core/utils/lock_map.h>
#include <list>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif
class Mutex;
class WaitCondition;
class ThreadInitializer;
class ThreadFinalizer;

class ThreadManager
: public ThreadCollector,
  public BlockedTimingExecutor
{
 public:
  ThreadManager();
  ThreadManager(ThreadInitializer *initializer, ThreadFinalizer *finalizer);
  virtual ~ThreadManager();

  void set_inifin(ThreadInitializer *initializer,
		  ThreadFinalizer *finalizer);

  virtual void add(ThreadList &tl)
  {
    add_maybelocked(tl, /* lock */ true);
  }

  virtual void add(Thread *t)
  {
    add_maybelocked(t, /* lock */ true);
  }

  virtual void remove(ThreadList &tl)
  {
    remove_maybelocked(tl, /* lock */ true);
  }

  virtual void remove(Thread *t)
  {
    remove_maybelocked(t, /* lock */ true);
  }

  virtual void force_remove(ThreadList &tl);
  virtual void force_remove(Thread *t);

  virtual void wakeup_and_wait(BlockedTimingAspect::WakeupHook hook,
			       unsigned int timeout_usec = 0);
  virtual void wakeup(BlockedTimingAspect::WakeupHook hook,
		      Barrier *barrier = 0);
  virtual void try_recover(std::list<std::string> &recovered_threads);

  virtual bool timed_threads_exist();
  virtual void wait_for_timed_threads();
  virtual void interrupt_timed_thread_wait();

  ThreadCollector *  aspect_collector() const;

 private:
  void internal_add_thread(Thread *t);
  void internal_remove_thread(Thread *t);
  void add_maybelocked(ThreadList &tl, bool lock);
  void add_maybelocked(Thread *t, bool lock);
  void remove_maybelocked(ThreadList &tl, bool lock);
  void remove_maybelocked(Thread *t, bool lock);

  class ThreadManagerAspectCollector : public ThreadCollector
  {
   public:
    ThreadManagerAspectCollector(ThreadManager *parent_manager);

    virtual void add(ThreadList &tl);
    virtual void add(Thread *t);

    virtual void remove(ThreadList &tl);
    virtual void remove(Thread *t);

    virtual void force_remove(ThreadList &tl);
    virtual void force_remove(Thread *t);

   private:
    ThreadManager *__parent_manager;
  };

 private:
  ThreadInitializer *__initializer;
  ThreadFinalizer   *__finalizer;

  LockMap< BlockedTimingAspect::WakeupHook, ThreadList > __threads;
  LockMap< BlockedTimingAspect::WakeupHook, ThreadList >::iterator __tit;

  ThreadList     __untimed_threads;
  WaitCondition *__waitcond_timedthreads;

  ThreadManagerAspectCollector *__aspect_collector;
  bool __interrupt_timed_thread_wait;

};

} // end namespace fawkes

#endif
