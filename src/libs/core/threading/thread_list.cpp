
/***************************************************************************
 *  thread_list.cpp - Thread list
 *
 *  Created: Tue Oct 31 18:20:59 2006
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

#include <core/threading/thread_list.h>
#include <core/threading/thread.h>
#include <core/threading/thread_initializer.h>
#include <core/threading/thread_finalizer.h>
#include <core/threading/mutex.h>
#include <core/threading/read_write_lock.h>

#include <cstring>

/** @class ThreadListSealedException <core/threading/thread_list.h>
 * Thread list sealed exception.
 * This exception is thrown whenever you execute an action that would
 * modify the thread list like adding or removing elements on a
 * sealed list. A list can only be sealed and never be unsealed afterwards.
 * This exception is meant to be only thrown by ThreadList.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param operation operation that failed
 */
ThreadListSealedException::ThreadListSealedException(const char *operation)
  : Exception("ThreadList is sealed")
{
  append("Operation '%s' is not allowed on a sealed thread list");
}


/** @class ThreadListNotSealedException <core/threading/thread_list.h>
 * Thread list not sealed exception.
 * This exception is thrown whenever the thread list is given to some
 * method that expects a sealed list (probably because it sealed the
 * list by itself).
 * This exception is meant to be only thrown by users of ThreadList.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param msg descriptive message of failed operation
 */
ThreadListNotSealedException::ThreadListNotSealedException(const char *msg)
  : Exception("ThreadList is *not* sealed")
{
  append(msg);
}


/** @class ThreadList core/threading/thread_list.h
 * List of threads.
 * This is a list of threads derived from stl::list. It features special
 * wakeup methods that will wakeup all threads in the list. The list can
 * and must be locked in iterator operations and when adding or deleting
 * elements from the list.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param tlname optional name which is used for better readable error
 * messages.
 */
ThreadList::ThreadList(const char *tlname)
{
  _name = strdup(tlname);
  _sealed = false;
  _finalize_mutex = new Mutex();
  _sync_lock = new ReadWriteLock();
  clear();
}


/** Copy constructor.
 * @param tl thread list to copy
 */
ThreadList::ThreadList(const ThreadList &tl)
  : LockList<Thread *>(tl)
{
  _name = strdup(tl._name);
  _sealed = tl._sealed;
  _finalize_mutex = new Mutex();
  _sync_lock = new ReadWriteLock();
}


/** Destructor. */
ThreadList::~ThreadList()
{
  free(_name);
  delete _sync_lock;
  delete _finalize_mutex;
}


/** Wakeup all threads in list. */
void
ThreadList::wakeup()
{
  lock();
  for (iterator i = begin(); i != end(); ++i) {
    (*i)->wakeup();
  }
  unlock();
}


/** Wakeup all threads in list and have them wait for the barrier.
 * @param barrier Barrier to wait for after loop
 */
void
ThreadList::wakeup(Barrier *barrier)
{
  lock();
  for (iterator i = begin(); i != end(); ++i) {
    (*i)->wakeup(barrier);
  }
  unlock();
}



/** Initialize threads.
 * The threads are being initialized.
 * This operation is carried out unlocked. Lock it from the outside if needed.
 * This is done because it is likely that this will be chained with other
 * actions that require locking, thus you can lock the whole operation.
 * @param initializer thread initializer to use
 */
void
ThreadList::init(ThreadInitializer *initializer)
{
  for (ThreadList::iterator i = begin(); i != end(); ++i) {
    try {
      initializer->init(*i);
    } catch (CannotInitializeThreadException &e) {
      e.append("Initializing thread in list '%s' failed", _name);
      throw;
    }
  }
}

/** Start threads.
 * The threads are started.
 * This operation is carried out unlocked. Lock it from the outside if needed.
 * This is done because it is likely that this will be chained with other
 * actions that require locking, thus you can lock the whole operation.
 */
void
ThreadList::start()
{
  for (iterator i = begin(); i != end(); ++i) {
    (*i)->start();
  }
}


/** Cancel threads.
 * The threads are canceled.
 * This operation is carried out unlocked. Lock it from the outside if needed.
 * This is done because it is likely that this will be chained with other
 * actions that require locking, thus you can lock the whole operation.
 *
 * This is especially handy for detached threads. Since errorneous behavior
 * has been seen when run inside gdb something like
 * @code
 * tl.cancel();
 * tl.join();
 * @endcode
 * shout be avoided. Instead use
 * @code
 * tl.stop();
 * @endcode
 */
void
ThreadList::cancel()
{
  for (iterator i = begin(); i != end(); ++i) {
    (*i)->cancel();
  }
}


/** Join threads.
 * The threads are joined.
 * This operation is carried out unlocked. Lock it from the outside if needed.
 * This is done because it is likely that this will be chained with other
 * actions that require locking, thus you can lock the whole operation.
 *
 * Since errorneous behavior
 * has been seen when run inside gdb something like
 * @code
 * tl.cancel();
 * tl.join();
 * @endcode
 * shout be avoided. Instead use
 * @code
 * tl.stop();
 * @endcode
 */
void
ThreadList::join()
{
  for (iterator i = begin(); i != end(); ++i) {
    (*i)->join();
  }
}


/** Stop threads.
 * The threads are canceled and joined.
 * This operation is carried out unlocked. Lock it from the outside if needed.
 * This is done because it is likely that this will be chained with other
 * actions that require locking, thus you can lock the whole operation.
 */
void
ThreadList::stop()
{
  for (iterator i = begin(); i != end(); ++i) {
    (*i)->cancel();
    (*i)->join();
  }
}


/** Prepare finalize.
 * The threads are prepared for finalization. If any of the threads return
 * false the whole list will return false.
 * This operation is carried out unlocked. Lock it from the outside if needed.
 * This is done because it is likely that this will be chained with other
 * actions that require locking, thus you can lock the whole operation.
 * @return true, if prepare_finalize() returned true for all threads in the
 * list, false if at least one thread returned false.
 */
bool
ThreadList::prepare_finalize()
{
  _finalize_mutex->lock();
  _sync_lock->lockForWrite();
  bool can_finalize = true;
  for (iterator i = begin(); i != end(); ++i) {
    try {
      if ( ! (*i)->prepare_finalize() ) {
	can_finalize = false;
      }
    } catch (CannotFinalizeThreadException &e) {
      e.append("Thread '%s' throw an exception while preparing finalization of "
	       "ThreadList '%s'", (*i)->name(), _name);
      throw;
    }
  }
  _finalize_mutex->unlock();
  return can_finalize;
}




/** Finalize Threads.
 * The threads are finalized.
 * This operation is carried out unlocked. Lock it from the outside if needed.
 * This is done because it is likely that this will be chained with other
 * actions that require locking, thus you can lock the whole operation.
 * @param finalizer thread finalize to use to finalize the threads.
 */
void
ThreadList::finalize(ThreadFinalizer *finalizer)
{
  bool error = false;
  Exception me("One or more threads failed to finalize");
  for (iterator i = begin(); i != end(); ++i) {
    try {
      finalizer->finalize(*i);
    } catch (CannotFinalizeThreadException &e) {
      error = true;
      me.append("Could not finalize thread '%s' in list '%s'", (*i)->name(), _name);
      me.append(e);
    }
  }
  if ( error ) {
    throw me;
  }
}


/** Cancel finalization on all threads.
 */
void
ThreadList::cancel_finalize()
{
  _finalize_mutex->lock();
  for (iterator i = begin(); i != end(); ++i) {
    (*i)->cancel_finalize();
  }
  _sync_lock->unlock();
  _finalize_mutex->unlock();
}


/** Force stop of all threads.
 * This will call prepare_finalize(), finalize(), cancel() and join() on the
 * list without caring about the return values in the prepare_finalize() step.
 * @param finalizer thread finalize to use to finalize the threads.
 */
void
ThreadList::force_stop(ThreadFinalizer *finalizer)
{
  try {
    prepare_finalize();
    finalize(finalizer);
    stop();
  } catch (Exception &e) {
    // ignored
  }
}


/** Name of the thread list.
 * This can be used for better log output to identify the list that causes
 * problems.
 * @return name of thread list
 */
const char *
ThreadList::name()
{
  return _name;
}


/** Check if list is sealed.
 * If the list is sealed, no more writing operations are allowed and will trigger
 * an exception.
 * @return true, if list is sealed, false otherwise
 */
bool
ThreadList::sealed()
{
  return _sealed;
}


/** Seal the list. */
void
ThreadList::seal()
{
  _sealed = true;
}


/** Add thread to the front.
 * Add thread to the beginning of the list.
 * @param thread thread to add
 */
void
ThreadList::push_front(Thread *thread)
{
  if ( _sealed ) throw ThreadListSealedException("push_front");

  if (thread->opmode() != Thread::OPMODE_CONTINUOUS )
    thread->set_threadlist_sync_lock(_sync_lock);
  LockList<Thread *>::push_front(thread);
}


/** Add thread to the front with lock protection.
 * Add thread to the beginning of the list. The operation is protected
 * by the thread list lock.
 * @param thread thread to add
 */
void
ThreadList::push_front_locked(Thread *thread)
{
  if ( _sealed ) throw ThreadListSealedException("push_front_locked");

  if (thread->opmode() != Thread::OPMODE_CONTINUOUS )
    thread->set_threadlist_sync_lock(_sync_lock);
  LockList<Thread *>::push_front_locked(thread);
}


/** Add thread to the end.
 * Add thread to the end of the list.
 * @param thread thread to add
 */
void
ThreadList::push_back(Thread *thread)
{
  if ( _sealed ) throw ThreadListSealedException("push_back");

  if (thread->opmode() != Thread::OPMODE_CONTINUOUS )
    thread->set_threadlist_sync_lock(_sync_lock);
  LockList<Thread *>::push_back(thread);
}


/** Add thread to the end with lock protection.
 * Add thread to the end of the list. The operation is protected
 * by the thread list lock.
 * @param thread thread to add
 */
void
ThreadList::push_back_locked(Thread *thread)
{
  if ( _sealed ) throw ThreadListSealedException("push_back_locked");

  if (thread->opmode() != Thread::OPMODE_CONTINUOUS )
    thread->set_threadlist_sync_lock(_sync_lock);
  LockList<Thread *>::push_back_locked(thread);
}


/** Clear the list.
 * Removes all elements.
 */
void
ThreadList::clear()
{
  if ( _sealed ) throw ThreadListSealedException("clear");

  LockList<Thread *>::clear();
}


/** Remove with lock protection.
 * @param thread thread to remove.
 */
void
ThreadList::remove(Thread *thread)
{
  if ( _sealed ) throw ThreadListSealedException("remove_locked");

  if (thread->opmode() != Thread::OPMODE_CONTINUOUS )
    thread->set_threadlist_sync_lock(NULL);
  LockList<Thread *>::remove(thread);
}


/** Remove with lock protection.
 * @param thread thread to remove.
 */
void
ThreadList::remove_locked(Thread *thread)
{
  if ( _sealed ) throw ThreadListSealedException("remove_locked");

  if (thread->opmode() != Thread::OPMODE_CONTINUOUS )
    thread->set_threadlist_sync_lock(NULL);
  LockList<Thread *>::remove_locked(thread);
}


/** Remove first element. */
void
ThreadList::pop_front()
{
  if ( _sealed ) throw ThreadListSealedException("pop_front");

  if (front()->opmode() != Thread::OPMODE_CONTINUOUS )
    front()->set_threadlist_sync_lock(NULL);
  LockList<Thread *>::pop_front();
}


/** Remove last element. */
void
ThreadList::pop_back()
{
  if ( _sealed ) throw ThreadListSealedException("pop_back");

  if (back()->opmode() != Thread::OPMODE_CONTINUOUS )
    back()->set_threadlist_sync_lock(NULL);
  LockList<Thread *>::pop_back();
}


/** Erase element at given position.
 * @param pos iterator marking the element to remove.
 * @return iterator to element that follows pos
 */
ThreadList::iterator
ThreadList::erase(iterator pos)
{
  if ( _sealed ) throw ThreadListSealedException("erase");

  if ((*pos)->opmode() != Thread::OPMODE_CONTINUOUS )
    (*pos)->set_threadlist_sync_lock(NULL);
  return LockList<Thread *>::erase(pos);
}
