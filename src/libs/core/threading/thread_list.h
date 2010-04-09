
/***************************************************************************
 *  thread_list.h - Thread list
 *
 *  Created: Tue Oct 31 18:04:31 2006
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

#ifndef __CORE_THREADING_THREAD_LIST_H_
#define __CORE_THREADING_THREAD_LIST_H_

#include <core/exception.h>
#include <core/threading/thread.h>
#include <core/threading/thread_initializer.h>
#include <core/threading/thread_finalizer.h>
#include <core/utils/lock_list.h>

#include <utility>
#include <string>

namespace fawkes {


class ThreadList;
class Mutex;
class Barrier;
class InterruptibleBarrier;

class ThreadListSealedException : public Exception
{
 public:
  ThreadListSealedException(const char *operation);
};

class ThreadListNotSealedException : public Exception
{
 public:
  ThreadListNotSealedException(const char *format, ...);
};


class ThreadList : private LockList<Thread *>
{
 public:
  ThreadList(const char *tlname = "");
  ThreadList(bool maintain_barrier, const char *tlname = "");
  ThreadList(const ThreadList &tl);
  ~ThreadList();

  const char *  name();
  void          set_name(const char *format, ...);

  void seal();
  bool sealed();

  void init(ThreadInitializer *initializer, ThreadFinalizer *finalizer);
  bool prepare_finalize(ThreadFinalizer *finalizer);
  void finalize(ThreadFinalizer *finalizer);
  void cancel_finalize();
  void set_prepfin_hold(bool hold);

  void wakeup();
  void wakeup(Barrier *barrier);
  void wakeup_unlocked();
  void wakeup_unlocked(Barrier *barrier);
  void wakeup_and_wait(unsigned int timeout_sec = 0,
		       unsigned int timeout_nanosec = 0);
  void start();
  void stop();
  void cancel();
  void join();

  void try_recover(std::list<std::string> &recovered_threads);
  void set_maintain_barrier(bool maintain_barrier);

  void force_stop(ThreadFinalizer *finalizer);

  void push_front(Thread *thread);
  void push_front_locked(Thread *thread);
  void push_back(Thread *thread);
  void push_back_locked(Thread *thread);
  void clear();
  void pop_back();
  void pop_front();
  ThreadList::iterator erase(iterator pos);

  void remove(Thread *thread);
  void remove_locked(Thread *thread);

  using LockList<Thread *>::begin;
  using LockList<Thread *>::end;
  using LockList<Thread *>::iterator;
  using LockList<Thread *>::lock;
  using LockList<Thread *>::try_lock;
  using LockList<Thread *>::unlock;
  using LockList<Thread *>::size;
  using LockList<Thread *>::empty;
  using LockList<Thread *>::front;
  using LockList<Thread *>::back;

  ThreadList &  operator=  (const ThreadList & tl);


 private:
  void notify_of_failed_init();
  void update_barrier();

 private:
  char                   *__name;
  bool                    __sealed;
  Mutex                  *__finalize_mutex;
  InterruptibleBarrier   *__wnw_barrier;

  std::list<std::pair<InterruptibleBarrier *, ThreadList> >  __wnw_bad_barriers;
  std::list<std::pair<InterruptibleBarrier *, ThreadList> >::iterator __wnw_bbit;
};


} // end namespace fawkes

#endif
