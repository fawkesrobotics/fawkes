
/***************************************************************************
 *  thread_manager.h - Fawkes thread manager
 *
 *  Created: Thu Nov  3 19:08:23 2006 (on train to Cologne)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __FAWKES_THREAD_MANAGER_H_
#define __FAWKES_THREAD_MANAGER_H_

#include <core/threading/thread_list.h>
#include <core/threading/thread_collector.h>
#include <core/exception.h>
#include <aspect/blocked_timing.h>

#include <core/utils/lock_map.h>
#include <list>

class Mutex;
class WaitCondition;
class ThreadInitializer;
class ThreadFinalizer;

class FawkesThreadManager : public ThreadCollector
{
 public:
  FawkesThreadManager();
  virtual ~FawkesThreadManager();

  void set_inifin(ThreadInitializer *initializer, ThreadFinalizer *finalizer);

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

  void wakeup_and_wait(BlockedTimingAspect::WakeupHook hook);

  bool timed_threads_exist() const;
  void wait_for_timed_threads();

  ThreadCollector *  aspect_collector() const;

 private:
  void internal_add_thread(Thread *t);
  void internal_remove_thread(Thread *t);
  void add_maybelocked(ThreadList &tl, bool lock);
  void add_maybelocked(Thread *t, bool lock);
  void remove_maybelocked(ThreadList &tl, bool lock);
  void remove_maybelocked(Thread *t, bool lock);

  class FawkesThreadManagerAspectCollector : public ThreadCollector
  {
   public:
    FawkesThreadManagerAspectCollector(FawkesThreadManager *parent_manager);

    virtual void add(ThreadList &tl);
    virtual void add(Thread *t);

    virtual void remove(ThreadList &tl);
    virtual void remove(Thread *t);

   private:
    FawkesThreadManager *__parent_manager;
  };

 private:
  ThreadInitializer *initializer;
  ThreadFinalizer   *finalizer;

  LockMap< BlockedTimingAspect::WakeupHook, ThreadList > threads;
  LockMap< BlockedTimingAspect::WakeupHook, ThreadList >::iterator tit;

  ThreadList untimed_threads;
  WaitCondition *waitcond_timedthreads;

  FawkesThreadManagerAspectCollector *__aspect_collector;
};

#endif
