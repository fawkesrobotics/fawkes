
/***************************************************************************
 *  thread_manager.h - Fawkes thread manager
 *
 *  Created: Thu Nov  3 19:08:23 2006 (on train to Cologne)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
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
class Barrier;
class WaitCondition;
class ThreadInitializer;
class ThreadFinalizer;

class FawkesThreadManager : public ThreadCollector
{
 public:
  FawkesThreadManager();
  virtual ~FawkesThreadManager();

  void set_inifin(ThreadInitializer *initializer, ThreadFinalizer *finalizer);

  virtual void add(ThreadList &tl);
  virtual void add(Thread *t);

  virtual void remove(ThreadList &tl);
  virtual void remove(Thread *t);

  virtual void force_remove(ThreadList &tl);
  virtual void force_remove(Thread *t);

  void wakeup_and_wait(BlockedTimingAspect::WakeupHook hook);

  bool timed_threads_exist() const;
  void wait_for_timed_threads();


 private:
  void internal_add_thread(Thread *t);
  void internal_remove_thread(Thread *t);
  void update_barrier(BlockedTimingAspect::WakeupHook hook);

 private:
  ThreadInitializer *initializer;
  ThreadFinalizer   *finalizer;

  LockMap< BlockedTimingAspect::WakeupHook, ThreadList > threads;
  LockMap< BlockedTimingAspect::WakeupHook, ThreadList >::iterator tit;

  std::map< BlockedTimingAspect::WakeupHook, Barrier * >  barriers;

  ThreadList untimed_threads;
  WaitCondition *wait_for_timed;
};

#endif
