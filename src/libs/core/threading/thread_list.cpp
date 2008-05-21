
/***************************************************************************
 *  thread_list.cpp - Thread list
 *
 *  Created: Tue Oct 31 18:20:59 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <core/threading/thread_list.h>
#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/threading/barrier.h>
#include <core/threading/read_write_lock.h>
#include <core/exceptions/software.h>

#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

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
  append("Operation '%s' is not allowed on a sealed thread list", operation);
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
 * @param format format of message
 */
ThreadListNotSealedException::ThreadListNotSealedException(const char *format, ...)
  : Exception()
{
  va_list va;
  va_start(va, format);
  append_va(format, va);
  va_end(va);
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
  _wnw_barrier = NULL;
  clear();
}


/** Constructor.
 * @param maintain_barrier if true, an internal barrier is maintained during add and
 * remove operations such that wakeup_and_wait() can be used.
 * @param tlname optional name which is used for better readable error
 * messages.
 */
ThreadList::ThreadList(bool maintain_barrier, const char *tlname)
{
  _name = strdup(tlname);
  _sealed = false;
  _finalize_mutex = new Mutex();
  _sync_lock = new ReadWriteLock();
  if ( maintain_barrier ) {
    _wnw_barrier = new Barrier(1);
  } else {
    _wnw_barrier = NULL;
  }
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
  if ( tl._wnw_barrier != NULL ) {
    _wnw_barrier = new Barrier(1);
  } else {
    _wnw_barrier = NULL;
  }
}


/** Destructor. */
ThreadList::~ThreadList()
{
  free(_name);
  delete _sync_lock;
  delete _finalize_mutex;
  delete _wnw_barrier;
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


/** Wakeup all threads in list.
 * This method wakes up all thread without acquiring the lock first.
 * This method must only be used if the thread list is locked otherwise!
 */
void
ThreadList::wakeup_unlocked()
{
  for (iterator i = begin(); i != end(); ++i) {
    (*i)->wakeup();
  }
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


/** Wakeup all threads in list and have them wait for the barrier.
 * This method wakes up all thread without aquiring the lock first.
 * This method must only be used if the thread list is locked otherwise!
 * @param barrier Barrier to wait for after loop
 */
void
ThreadList::wakeup_unlocked(Barrier *barrier)
{
  for (iterator i = begin(); i != end(); ++i) {
    (*i)->wakeup(barrier);
  }
}


/** Wakeup threads and wait for them to finish.
 * This assumes that all threads are in wait-for-wakeup mode. The threads are woken
 * up with an internally maintained barrier. The method will return when all threads
 * have finished one loop() iteration.
 * @exception NullPointerException thrown, if no internal barrier is maintained. Make sure
 * you use the proper constructor.
 */
void
ThreadList::wakeup_and_wait()
{
  if ( ! _wnw_barrier ) {
    throw NullPointerException("ThreadList::wakeup_and_wait() can only be called if "
			       "barrier is maintained");
  }
  lock();
  wakeup_unlocked(_wnw_barrier);
  _wnw_barrier->wait();
  unlock();
}


/** Set if this thread list should maintain a barrier.
 * This operation does an implicit locking of the list.
 * @param maintain_barrier true to maintain an internal barrier, false to disable it.
 */
void
ThreadList::set_maintain_barrier(bool maintain_barrier)
{
  lock();
  if ( maintain_barrier ) {
    if ( ! _wnw_barrier ) {
      _wnw_barrier = new Barrier(size() + 1);
    }
  } else {
    delete _wnw_barrier;
    _wnw_barrier = NULL;
  }
  unlock();
}


/** Initialize threads.
 * The threads are being initialized.
 * This operation is carried out unlocked. Lock it from the outside if needed.
 * This is done because it is likely that this will be chained with other
 * actions that require locking, thus you can lock the whole operation.
 * @param initializer thread initializer to use
 * @param finalizer finalizer to use to finalize threads that have been successfully
 * initialized before one thread failed.
 * @exception CannotInitializeThreadException thrown if at least one of the
 * threads in this list could not be initialized.
 */
void
ThreadList::init(ThreadInitializer *initializer, ThreadFinalizer *finalizer)
{
  CannotInitializeThreadException cite("Cannot initialize one or more threads");
  ThreadList initialized_threads;
  bool success = true;
  for (ThreadList::iterator i = begin(); i != end(); ++i) {
    try {
      initializer->init(*i);
      (*i)->init();
    } catch (CannotInitializeThreadException &e) {
      notify_of_failed_init();
      cite.append(e);
      cite.prepend("Initializing thread in list '%s' failed", _name);
      success = false;
      break;
    } catch (Exception &e) {
      notify_of_failed_init();
      cite.append(e);
      cite.prepend("Could not initialize thread '%s'", (*i)->name());
      success = false;
      break;
    } catch (...) {
      notify_of_failed_init();
      cite.append("Could not initialize thread '%s'", (*i)->name());
      cite.prepend("Unknown exception caught");
      success = false;
      break;
    }
  }

  if ( ! success ) {
    initialized_threads.finalize(finalizer);
    throw cite;
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
  for (reverse_iterator i = rbegin(); i != rend(); ++i) {
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
 * @param finalizer thread finalizer to use to prepare finalization of the threads
 * @return true, if prepare_finalize() returned true for all threads in the
 * list, false if at least one thread returned false.
 */
bool
ThreadList::prepare_finalize(ThreadFinalizer *finalizer)
{
  _finalize_mutex->lock();
  _sync_lock->lock_for_write();
  bool can_finalize = true;
  CannotFinalizeThreadException cfte("Cannot finalize one or more threads");
  bool threw_exception = false;
  for (reverse_iterator i = rbegin(); i != rend(); ++i) {
    // Note that this loop may NOT be interrupted in the middle by break,
    // since even if the thread denies finalization it can still be finalized
    // and we have to ensure that every thread got a call to prepare_finalize()!
    try {
      if ( ! finalizer->prepare_finalize(*i) ) {
	can_finalize = false;
      }
      if ( ! (*i)->prepare_finalize() ) {
	can_finalize = false;
      }
    } catch (CannotFinalizeThreadException &e) {
      cfte.append("Thread '%s' throw an exception while preparing finalization of "
		  "ThreadList '%s'", (*i)->name(), _name);
      threw_exception = true;
    }
  }
  _sync_lock->unlock();
  _finalize_mutex->unlock();
  if ( threw_exception ) {
    throw cfte;
  }
  return can_finalize;
}




/** Finalize Threads.
 * The threads are finalized.
 * This operation is carried out unlocked. Lock it from the outside if needed.
 * This is done because it is likely that this will be chained with other
 * actions that require locking, thus you can lock the whole operation.
 * @param finalizer thread finalizer to use to finalize the threads
 */
void
ThreadList::finalize(ThreadFinalizer *finalizer)
{
  bool error = false;
  Exception me("One or more threads failed to finalize");
  for (reverse_iterator i = rbegin(); i != rend(); ++i) {
    try {
      finalizer->finalize(*i);
    } catch (CannotFinalizeThreadException &e) {
      error = true;
      me.append("Could not finalize thread '%s' in list '%s'", (*i)->name(), _name);
      me.append(e);
    }
    try {
      (*i)->finalize();
    } catch (CannotFinalizeThreadException &e) {
      error = true;
      me.append("AspectIniFin called Thread[%s]::finalize() which failed", (*i)->name());
      me.append(e);
    } catch (Exception &e) {
      me.append("AspectIniFin called Thread[%s]::finalize() which failed", (*i)->name());
      me.append(e);
    } catch (...) {
      me.append("Thread[%s]::finalize() threw unsupported exception", (*i)->name());
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
  for (reverse_iterator i = rbegin(); i != rend(); ++i) {
    (*i)->cancel_finalize();
  }
  _finalize_mutex->unlock();
}


/** Set prepfin hold on all threads.
 * This method will call Thread::set_prepfin_hold() for all threads in the list. If
 * any of the threads fails to set prepfin hold then all thread were it has already
 * been set are set to prepfin hold false.
 * @param hold prepfin hold value
 * @see Thread::set_prepfin_hold()
 */
void
ThreadList::set_prepfin_hold(bool hold)
{
  iterator i;
  try {
    for (i = begin(); i != end(); ++i) {
      (*i)->set_prepfin_hold(hold);
    }
  } catch (Exception &e) {
    // boom, we failed, at least one thread was already in the state of being prepared
    // for finalization, rollback the hold for the threads were we already set it
    for (iterator j = begin(); j != i; ++j) {
      (*j)->set_prepfin_hold(false);
    }
    throw;
  }
}


/** Force stop of all threads.
 * This will call prepare_finalize(), finalize(), cancel() and join() on the
 * list without caring about the return values in the prepare_finalize() step.
 * @param finalizer thread finalizer to use to finalize the threads.
 */
void
ThreadList::force_stop(ThreadFinalizer *finalizer)
{
  try {
    prepare_finalize(finalizer);
    stop();
    finalize(finalizer);
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


/** Set name of thread.
 * Use parameters similar to printf().
 * @param format format string
 */
void
ThreadList::set_name(const char *format, ...)
{
  va_list va;
  va_start(va, format);
  free(_name);
  vasprintf(&_name, format, va);
  va_end(va);
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
    thread->set_finalize_sync_lock(_sync_lock);
  LockList<Thread *>::push_front(thread);

  if ( _wnw_barrier)  update_barrier();
}


/** Add thread to the front with lock protection.
 * Add thread to the beginning of the list. The operation is protected
 * by the thread list lock.
 * The operation will succeed without blocking even
 * if the list is currently locked. It will push the thread to an internal temporary
 * list and will add the thread finally when the list is unlocked.
 * @param thread thread to add
 */
void
ThreadList::push_front_locked(Thread *thread)
{
  if ( _sealed ) throw ThreadListSealedException("push_front_locked");

  lock();
  if (thread->opmode() != Thread::OPMODE_CONTINUOUS )
    thread->set_finalize_sync_lock(_sync_lock);
  LockList<Thread *>::push_front(thread);
  
  if ( _wnw_barrier)  update_barrier();
  unlock();
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
    thread->set_finalize_sync_lock(_sync_lock);
  LockList<Thread *>::push_back(thread);

  if ( _wnw_barrier)  update_barrier();
}


/** Add thread to the end with lock protection.
 * Add thread to the end of the list. The operation is protected
 * by the thread list lock.
 * The operation will succeed without blocking even
 * if the list is currently locked. It will push the thread to an internal temporary
 * list and will add the thread finally when the list is unlocked.
 * @param thread thread to add
 */
void
ThreadList::push_back_locked(Thread *thread)
{
  if ( _sealed ) throw ThreadListSealedException("push_back_locked");

  if (thread->opmode() != Thread::OPMODE_CONTINUOUS )
    thread->set_finalize_sync_lock(_sync_lock);

  lock();
  LockList<Thread *>::push_back(thread);
  if ( _wnw_barrier)  update_barrier();
  unlock();
}


/** Clear the list.
 * Removes all elements.
 */
void
ThreadList::clear()
{
  if ( _sealed ) throw ThreadListSealedException("clear");

  LockList<Thread *>::clear();
  if ( _wnw_barrier)  update_barrier();
}


/** Remove with lock protection.
 * @param thread thread to remove.
 */
void
ThreadList::remove(Thread *thread)
{
  if ( _sealed ) throw ThreadListSealedException("remove_locked");

  if (thread->opmode() != Thread::OPMODE_CONTINUOUS )
    thread->set_finalize_sync_lock(NULL);
  LockList<Thread *>::remove(thread);

  if ( _wnw_barrier)  update_barrier();
}


/** Remove with lock protection.
 * @param thread thread to remove.
 */
void
ThreadList::remove_locked(Thread *thread)
{
  if ( _sealed ) throw ThreadListSealedException("remove_locked");

  if (thread->opmode() != Thread::OPMODE_CONTINUOUS )
    thread->set_finalize_sync_lock(NULL);

  lock();
  LockList<Thread *>::remove(thread);
  if ( _wnw_barrier)  update_barrier();
  unlock();
}


/** Remove first element. */
void
ThreadList::pop_front()
{
  if ( _sealed ) throw ThreadListSealedException("pop_front");

  if (front()->opmode() != Thread::OPMODE_CONTINUOUS )
    front()->set_finalize_sync_lock(NULL);
  LockList<Thread *>::pop_front();

  if ( _wnw_barrier)  update_barrier();
}


/** Remove last element. */
void
ThreadList::pop_back()
{
  if ( _sealed ) throw ThreadListSealedException("pop_back");

  if (back()->opmode() != Thread::OPMODE_CONTINUOUS )
    back()->set_finalize_sync_lock(NULL);
  LockList<Thread *>::pop_back();

  if ( _wnw_barrier)  update_barrier();
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
    (*pos)->set_finalize_sync_lock(NULL);
  ThreadList::iterator rv = LockList<Thread *>::erase(pos);

  if ( _wnw_barrier)  update_barrier();
  return rv;
}


/** Update internal barrier. */
void
ThreadList::update_barrier()
{
  delete _wnw_barrier;
  _wnw_barrier = new Barrier(size() + 1);
}


/** Notify all threads of failed init. */
void
ThreadList::notify_of_failed_init()
{
  for (ThreadList::iterator i = begin(); i != end(); ++i) {
    (*i)->notify_of_failed_init();
  }
}


} // end namespace fawkes
