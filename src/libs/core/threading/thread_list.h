
/***************************************************************************
 *  thread_list.h - Thread list
 *
 *  Created: Tue Oct 31 18:04:31 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __CORE_THREADING_THREAD_LIST_H_
#define __CORE_THREADING_THREAD_LIST_H_

#include <core/exception.h>
#include <core/utils/lock_list.h>

class Thread;
class Mutex;
class Barrier;
class ReadWriteLock;
class ThreadInitializer;
class ThreadFinalizer;

class ThreadListSealedException : public Exception
{
 public:
  ThreadListSealedException(const char *operation);
};

class ThreadListNotSealedException : public Exception
{
 public:
  ThreadListNotSealedException(const char *msg);
};

class ThreadList : private LockList<Thread *>
{
 public:
  ThreadList(const char *tlname = "");
  ThreadList(const ThreadList &tl);
  ~ThreadList();

  const char *  name();

  void seal();
  bool sealed();

  void init(ThreadInitializer *initializer);
  bool prepare_finalize();
  void finalize(ThreadFinalizer *finalizer);
  void cancel_finalize();

  void wakeup();
  void wakeup(Barrier *barrier);
  void start();
  void stop();
  void cancel();
  void join();

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
  using LockList<Thread *>::tryLock;
  using LockList<Thread *>::unlock;
  using LockList<Thread *>::size;
  using LockList<Thread *>::front;
  using LockList<Thread *>::back;

 private:
  char          *_name;
  bool           _sealed;
  Mutex         *_finalize_mutex;
  ReadWriteLock *_sync_lock;
};

#endif
