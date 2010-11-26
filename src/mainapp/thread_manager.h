
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

#ifndef __FAWKES_THREAD_MANAGER_H_
#define __FAWKES_THREAD_MANAGER_H_

#include <core/threading/thread_list.h>
#include <core/threading/thread_collector.h>
#include <core/exception.h>
#include <aspect/blocked_timing.h>
#include <aspect/blocked_timing/executor.h>

#include <core/utils/lock_map.h>
#include <list>

namespace fawkes {
  class Mutex;
  class WaitCondition;
  class ThreadInitializer;
  class ThreadFinalizer;
}

class FawkesThreadManager
: public fawkes::ThreadCollector,
  public fawkes::BlockedTimingExecutor
{
 public:
  FawkesThreadManager();
  virtual ~FawkesThreadManager();

  void set_inifin(fawkes::ThreadInitializer *initializer,
		  fawkes::ThreadFinalizer *finalizer);

  virtual void add(fawkes::ThreadList &tl)
  {
    add_maybelocked(tl, /* lock */ true);
  }

  virtual void add(fawkes::Thread *t)
  {
    add_maybelocked(t, /* lock */ true);
  }

  virtual void remove(fawkes::ThreadList &tl)
  {
    remove_maybelocked(tl, /* lock */ true);
  }

  virtual void remove(fawkes::Thread *t)
  {
    remove_maybelocked(t, /* lock */ true);
  }

  virtual void force_remove(fawkes::ThreadList &tl);
  virtual void force_remove(fawkes::Thread *t);

  virtual void wakeup_and_wait(fawkes::BlockedTimingAspect::WakeupHook hook,
			       unsigned int timeout_usec = 0);
  virtual void wakeup(fawkes::BlockedTimingAspect::WakeupHook hook,
		      fawkes::Barrier *barrier = 0);
  virtual void try_recover(std::list<std::string> &recovered_threads);

  virtual bool timed_threads_exist();
  virtual void wait_for_timed_threads();
  virtual void interrupt_timed_thread_wait();

  fawkes::ThreadCollector *  aspect_collector() const;

 private:
  void internal_add_thread(fawkes::Thread *t);
  void internal_remove_thread(fawkes::Thread *t);
  void add_maybelocked(fawkes::ThreadList &tl, bool lock);
  void add_maybelocked(fawkes::Thread *t, bool lock);
  void remove_maybelocked(fawkes::ThreadList &tl, bool lock);
  void remove_maybelocked(fawkes::Thread *t, bool lock);

  class FawkesThreadManagerAspectCollector : public fawkes::ThreadCollector
  {
   public:
    FawkesThreadManagerAspectCollector(FawkesThreadManager *parent_manager);

    virtual void add(fawkes::ThreadList &tl);
    virtual void add(fawkes::Thread *t);

    virtual void remove(fawkes::ThreadList &tl);
    virtual void remove(fawkes::Thread *t);

    virtual void force_remove(fawkes::ThreadList &tl);
    virtual void force_remove(fawkes::Thread *t);

   private:
    FawkesThreadManager *__parent_manager;
  };

 private:
  fawkes::ThreadInitializer *initializer;
  fawkes::ThreadFinalizer   *finalizer;

  fawkes::LockMap< fawkes::BlockedTimingAspect::WakeupHook, fawkes::ThreadList > threads;
  fawkes::LockMap< fawkes::BlockedTimingAspect::WakeupHook, fawkes::ThreadList >::iterator tit;

  fawkes::ThreadList untimed_threads;
  fawkes::WaitCondition *waitcond_timedthreads;

  FawkesThreadManagerAspectCollector *__aspect_collector;
  bool __interrupt_timed_thread_wait;

};

#endif
