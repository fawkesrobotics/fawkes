
/***************************************************************************
 *  thread_list.cpp - Thread list
 *
 *  Created: Tue Oct 31 18:20:59 2006
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

#include <core/threading/thread_list.h>
#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/barrier.h>
#include <core/threading/interruptible_barrier.h>
#include <core/exceptions/software.h>
#include <core/exceptions/system.h>

#include <string>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>

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

/** @class ThreadList <core/threading/thread_list.h>
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
  __name = strdup(tlname);
  __sealed = false;
  __finalize_mutex = new Mutex();
  __wnw_barrier = NULL;
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
  __name = strdup(tlname);
  __sealed = false;
  __finalize_mutex = new Mutex();
  __wnw_barrier = NULL;
  clear();
  if ( maintain_barrier)  update_barrier();
}


/** Copy constructor.
 * @param tl thread list to copy
 */
ThreadList::ThreadList(const ThreadList &tl)
  : LockList<Thread *>(tl)
{
  __name = strdup(tl.__name);
  __sealed = tl.__sealed;
  __finalize_mutex = new Mutex();
  __wnw_barrier = NULL;
  if ( tl.__wnw_barrier != NULL )  update_barrier();
}


/** Destructor. */
ThreadList::~ThreadList()
{
  free(__name);
  delete __finalize_mutex;
  delete __wnw_barrier;
}


/** Assignment operator.
 * @param tl thread list to assign
 * @return reference to this instance
 */
ThreadList &
ThreadList::operator= (const ThreadList &tl)
{
  LockList<Thread *>::operator=(tl);
  __name = strdup(tl.__name);
  __sealed = tl.__sealed;
  __finalize_mutex = new Mutex();
  __wnw_barrier = NULL;
  if ( tl.__wnw_barrier != NULL )  update_barrier();

  return *this;
}


/** Wakeup all threads in list. */
void
ThreadList::wakeup()
{
  MutexLocker lock(mutex());

  for (iterator i = begin(); i != end(); ++i) {
    (*i)->wakeup();
  }
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
  MutexLocker lock(mutex());

  for (iterator i = begin(); i != end(); ++i) {
    (*i)->wakeup(barrier);
  }
}


/** Wakeup all threads in list and have them wait for the barrier.
 * This method wakes up all thread without aquiring the lock first.
 * This method must only be used if the thread list is locked otherwise!
 * @param barrier Barrier to wait for after loop
 */
void
ThreadList::wakeup_unlocked(Barrier *barrier)
{
  Exception *exc = NULL;
  unsigned int count = 1;
  for (iterator i = begin(); i != end(); ++i) {
    if ( ! (*i)->flagged_bad() ) {
      try {
        (*i)->wakeup(barrier);
      } catch (Exception &e) {
        if (! exc) {
          exc = new Exception(e);
        } else {
          exc->append(e);
        }
      }
      ++count;
    }
  }
  if (exc) {
    Exception te(*exc);
    delete exc;
    throw te;
  }
  if (count != barrier->count()) {
    throw Exception("ThreadList(%s)::wakeup(): barrier has count (%u) different "
		    "from number of unflagged threads (%u)", __name, barrier->count(), count);
  }
}


/** Wakeup threads and wait for them to finish.
 * This assumes that all threads are in wait-for-wakeup mode. The threads are woken
 * up with an internally maintained barrier. The method will return when all threads
 * have finished one loop() iteration.
 * @param timeout_sec timeout in seconds
 * @param timeout_nanosec timeout in nanoseconds
 * @exception NullPointerException thrown, if no internal barrier is maintained. Make sure
 * you use the proper constructor.
 */
void
ThreadList::wakeup_and_wait(unsigned int timeout_sec, unsigned int timeout_nanosec)
{
  if ( ! __wnw_barrier ) {
    throw NullPointerException("ThreadList::wakeup_and_wait() can only be called if "
			       "barrier is maintained");
  }

  MutexLocker lock(mutex());

  try {
    wakeup_unlocked(__wnw_barrier);
  } catch (Exception &e) {
    throw;
  }
  if ( ! __wnw_barrier->wait(timeout_sec, timeout_nanosec) ) {
    // timeout, we have a bad thread, flag it
    RefPtr<ThreadList> passed_threads = __wnw_barrier->passed_threads();
    ThreadList bad_threads;
    for (iterator i = begin(); i != end(); ++i) {
      bool ok = false;
      for (iterator j = passed_threads->begin(); j != passed_threads->end(); ++j) {
	if (*j == *i) {
	  ok = true;
	  break;
	}
      }
      if (! ok) {
	bad_threads.push_back(*i);
	(*i)->set_flag(Thread::FLAG_BAD);
      }
    }

    __wnw_bad_barriers.push_back(make_pair(__wnw_barrier, bad_threads));

    __wnw_barrier = NULL;
    update_barrier();

    // Formulate exception
    std::string s;
    if ( bad_threads.size() > 1 ) {
      s = "Multiple threads did not finish in time, flagging as bad: ";
      for (iterator i = bad_threads.begin(); i != bad_threads.end(); ++i) {
	s += std::string((*i)->name()) + " ";
      }
    } else if (bad_threads.size() == 0) {
      s = "Timeout happened, but no bad threads recorded.";
    } else {
      throw Exception("Thread %s did not finish in time (max %f), flagging as bad",
		      bad_threads.front()->name(),
		      (float)timeout_sec + (float)timeout_nanosec / 1000000000.);
    }
    throw Exception("%s", s.c_str());
  }
}


/** Set if this thread list should maintain a barrier.
 * This operation does an implicit locking of the list.
 * @param maintain_barrier true to maintain an internal barrier, false to disable it.
 */
void
ThreadList::set_maintain_barrier(bool maintain_barrier)
{
  MutexLocker lock(mutex());

  delete __wnw_barrier;
  __wnw_barrier = NULL;
  if ( maintain_barrier )  update_barrier();
}


/** Check if any of the bad barriers recovered.
 * If the ThreadList maintains the barrier these may get bad if a thread does
 * not finish in time. This method will check all bad barriers if the bad threads
 * have recovered, and if so it will re-integrate the bad threads.
 * @param recovered_threads upon return the names of any threads that could be
 * recovered from a bad state have been added to the list.
 */
void
ThreadList::try_recover(std::list<std::string> &recovered_threads)
{
  MutexLocker lock(mutex());

  bool changed = false;
  __wnw_bbit = __wnw_bad_barriers.begin();
  while (__wnw_bbit != __wnw_bad_barriers.end()) {
    iterator i = __wnw_bbit->second.begin();
    while (i != __wnw_bbit->second.end()) {
      if ( (*i)->waiting() ) {
	// waiting means running() finished and the barrier has been passed
	recovered_threads.push_back((*i)->name());
	// it finally finished, re-integrate and hope that it does not bust again
	(*i)->unset_flag(Thread::FLAG_BAD);
	i = __wnw_bbit->second.erase(i);
	changed = true;
      } else {
	++i;
      }
    }
    if ( __wnw_bbit->second.empty() ) {
      delete __wnw_bbit->first;
      __wnw_bbit = __wnw_bad_barriers.erase(__wnw_bbit);
    } else {
      ++__wnw_bbit;
    }
  }
  if ( changed )  update_barrier();
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
  CannotInitializeThreadException cite;
  ThreadList initialized_threads;
  bool success = true;
  for (ThreadList::iterator i = begin(); i != end(); ++i) {
    // if initializer fails, we assume it handles finalization
    try {
      initializer->init(*i);
    } catch (Exception &e) {
      cite.append("Initialized failed to initialize thread '%s'", (*i)->name());
      cite.append(e);
      success = false;
      break;
    }
    // if the thread's init() method fails, we need to finalize that very
    // thread only with the finalizer, already initialized threads muts be
    // fully finalized
    try {
      (*i)->init();
      initialized_threads.push_back(*i);
    } catch (CannotInitializeThreadException &e) {
      notify_of_failed_init();
      cite.append("Initializing thread '%s' in list '%s' failed",
		  (*i)->name(), __name);
      cite.append(e);
      finalizer->finalize(*i);
      success = false;
      break;
    } catch (Exception &e) {
      notify_of_failed_init();
      cite.append("Could not initialize thread '%s'", (*i)->name());
      cite.append(e);
      finalizer->finalize(*i);
      success = false;
      break;
    } catch (std::exception &e) {
      notify_of_failed_init();
      cite.append("Could not initialize thread '%s'", (*i)->name());
      cite.append("Caught std::exception or derivative: %s", e.what());
      finalizer->finalize(*i);
      success = false;
      break;
    } catch (...) {
      notify_of_failed_init();
      cite.append("Could not initialize thread '%s'", (*i)->name());
      cite.append("Unknown exception caught");
      finalizer->finalize(*i);
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
    // Workaround for pthreads annoyance
    usleep(5000);
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
  MutexLocker lock(__finalize_mutex);

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
		  "ThreadList '%s'", (*i)->name(), __name);
      threw_exception = true;
    }
  }
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
      me.append("Could not finalize thread '%s' in list '%s'", (*i)->name(), __name);
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
  MutexLocker lock(__finalize_mutex);

  for (reverse_iterator i = rbegin(); i != rend(); ++i) {
    (*i)->cancel_finalize();
  }
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
  return __name;
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
  
  char *tmpname;
  if (vasprintf(&tmpname, format, va) != -1) {
    free(__name);
    __name = tmpname;
  } else {
    throw OutOfMemoryException("ThreadList::set_name(): vasprintf() failed");
  }
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
  return __sealed;
}


/** Seal the list. */
void
ThreadList::seal()
{
  __sealed = true;
}


/** Add thread to the front.
 * Add thread to the beginning of the list.
 * @param thread thread to add
 */
void
ThreadList::push_front(Thread *thread)
{
  if ( __sealed ) throw ThreadListSealedException("push_front");

  LockList<Thread *>::push_front(thread);
  if ( __wnw_barrier)  update_barrier();
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
  if ( __sealed ) throw ThreadListSealedException("push_front_locked");

  MutexLocker lock(mutex());
  LockList<Thread *>::push_front(thread);
  if ( __wnw_barrier)  update_barrier();
}


/** Add thread to the end.
 * Add thread to the end of the list.
 * @param thread thread to add
 */
void
ThreadList::push_back(Thread *thread)
{
  if ( __sealed ) throw ThreadListSealedException("push_back");

  LockList<Thread *>::push_back(thread);
  if ( __wnw_barrier)  update_barrier();
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
  if ( __sealed ) throw ThreadListSealedException("push_back_locked");

  MutexLocker lock(mutex());
  LockList<Thread *>::push_back(thread);
  if ( __wnw_barrier)  update_barrier();
}


/** Clear the list.
 * Removes all elements.
 */
void
ThreadList::clear()
{
  if ( __sealed ) throw ThreadListSealedException("clear");

  LockList<Thread *>::clear();
  if ( __wnw_barrier)  update_barrier();
}


/** Remove with lock protection.
 * @param thread thread to remove.
 */
void
ThreadList::remove(Thread *thread)
{
  if ( __sealed ) throw ThreadListSealedException("remove_locked");

  LockList<Thread *>::remove(thread);
  if ( __wnw_barrier)  update_barrier();
}


/** Remove with lock protection.
 * @param thread thread to remove.
 */
void
ThreadList::remove_locked(Thread *thread)
{
  if ( __sealed ) throw ThreadListSealedException("remove_locked");

  MutexLocker lock(mutex());
  LockList<Thread *>::remove(thread);
  if ( __wnw_barrier)  update_barrier();
}


/** Remove first element. */
void
ThreadList::pop_front()
{
  if ( __sealed ) throw ThreadListSealedException("pop_front");

  LockList<Thread *>::pop_front();
  if ( __wnw_barrier)  update_barrier();
}


/** Remove last element. */
void
ThreadList::pop_back()
{
  if ( __sealed ) throw ThreadListSealedException("pop_back");

  LockList<Thread *>::pop_back();
  if ( __wnw_barrier)  update_barrier();
}


/** Erase element at given position.
 * @param pos iterator marking the element to remove.
 * @return iterator to element that follows pos
 */
ThreadList::iterator
ThreadList::erase(iterator pos)
{
  if ( __sealed ) throw ThreadListSealedException("erase");

  ThreadList::iterator rv = LockList<Thread *>::erase(pos);
  if ( __wnw_barrier)  update_barrier();
  return rv;
}


/** Update internal barrier. */
void
ThreadList::update_barrier()
{
  unsigned int num = 1;
  for (iterator i = begin(); i != end(); ++i) {
    if (! (*i)->flagged_bad() )  ++num;
  }
  delete __wnw_barrier;
  __wnw_barrier = new InterruptibleBarrier(num);
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
