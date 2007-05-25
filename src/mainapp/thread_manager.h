
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

#include <map>
#include <list>

class Barrier;
class ThreadInitializer;
class ThreadFinalizer;

class FawkesThreadManager : public ThreadCollector
{
 public:
  FawkesThreadManager(ThreadInitializer *initializer,
		      ThreadFinalizer   *finalizer);
  virtual ~FawkesThreadManager();

  virtual void add(ThreadList &tl);
  virtual void add(Thread *t);

  virtual void remove(ThreadList &tl);
  virtual void remove(Thread *t);

  virtual void force_remove(ThreadList &tl);
  virtual void force_remove(Thread *t);

  void wakeup(BlockedTimingAspect::WakeupHook hook);
  void wait(BlockedTimingAspect::WakeupHook hook);

 private:
  void internal_add_thread(Thread *t, std::list<BlockedTimingAspect::WakeupHook> &changed);
  void internal_remove_thread(Thread *t, std::list<BlockedTimingAspect::WakeupHook> &changed);
  void update_barriers(std::list<BlockedTimingAspect::WakeupHook> &changed);

 private:
  ThreadInitializer *initializer;
  ThreadFinalizer   *finalizer;

  std::map< BlockedTimingAspect::WakeupHook, ThreadList > threads;
  std::map< BlockedTimingAspect::WakeupHook, ThreadList >::iterator tit;

  std::map< BlockedTimingAspect::WakeupHook, Barrier * >  barriers;

  ThreadList untimed_threads;
};

#endif
