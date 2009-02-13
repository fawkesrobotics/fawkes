
/***************************************************************************
 *  thread_manager.cpp - Thread manager
 *
 *  Generated: Thu Nov  3 19:11:31 2006 (on train to Cologne)
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

#include <mainapp/thread_manager.h>
#include <core/threading/thread.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/wait_condition.h>
#include <core/threading/thread_initializer.h>
#include <core/threading/thread_finalizer.h>
#include <core/exceptions/software.h>
#include <core/exceptions/system.h>

#include <aspect/blocked_timing.h>

using namespace fawkes;

/** @class FawkesThreadManager mainapp/thread_manager.h
 * Thread Manager.
 * This class provides a manager for the threads. Threads are memorized by
 * their wakeup hook. When the thread manager is deleted, all threads are
 * appropriately cancelled, joined and deleted. Thus the thread manager
 * can be used for "garbage collection" of threads.
 *
 * The thread manager allows easy wakeup of threads of a given wakeup hook.
 *
 * The thread manager needs a thread initializer. Each thread that is added
 * to the thread manager is initialized with this. The runtime type information
 * (RTTI) supplied by C++ can be used to initialize threads if appropriate
 * (if the thread has certain aspects that need special treatment).
 *
 * @author Tim Niemueller
 */

FawkesThreadManager::FawkesThreadManagerAspectCollector::FawkesThreadManagerAspectCollector(FawkesThreadManager *parent_manager)
{
  __parent_manager = parent_manager;
}


void
FawkesThreadManager::FawkesThreadManagerAspectCollector::add(ThreadList &tl)
{
  BlockedTimingAspect *timed_thread;

  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    if ( (timed_thread = dynamic_cast<BlockedTimingAspect *>(*i)) != NULL ) {
      throw IllegalArgumentException("ThreadProducerAspect may not add threads with BlockedTimingAspect");
    }
  }

  __parent_manager->add_maybelocked(tl, /* lock */ false);
}


void
FawkesThreadManager::FawkesThreadManagerAspectCollector::add(Thread *t)
{
  BlockedTimingAspect *timed_thread;

  if ( (timed_thread = dynamic_cast<BlockedTimingAspect *>(t)) != NULL ) {
    throw IllegalArgumentException("ThreadProducerAspect may not add threads with BlockedTimingAspect");
  }

  __parent_manager->add_maybelocked(t, /* lock */ false);
}


void
FawkesThreadManager::FawkesThreadManagerAspectCollector::remove(ThreadList &tl)
{
  BlockedTimingAspect *timed_thread;

  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    if ( (timed_thread = dynamic_cast<BlockedTimingAspect *>(*i)) != NULL ) {
      throw IllegalArgumentException("ThreadProducerAspect may not remove threads with BlockedTimingAspect");
    }
  }

  __parent_manager->remove_maybelocked(tl, /* lock */ false);
}


void
FawkesThreadManager::FawkesThreadManagerAspectCollector::remove(Thread *t)
{
  BlockedTimingAspect *timed_thread;

  if ( (timed_thread = dynamic_cast<BlockedTimingAspect *>(t)) != NULL ) {
    throw IllegalArgumentException("ThreadProducerAspect may not remove threads with BlockedTimingAspect");
  }

  __parent_manager->remove_maybelocked(t, /* lock */ false);
}


void
FawkesThreadManager::FawkesThreadManagerAspectCollector::force_remove(fawkes::ThreadList &tl)
{
  throw AccessViolationException("ThreadManagerAspect threads may not force removal of threads");
}

void
FawkesThreadManager::FawkesThreadManagerAspectCollector::force_remove(fawkes::Thread *t)
{
  throw AccessViolationException("ThreadManagerAspect threads may not force removal of threads");
}


/** Constructor.
 */
FawkesThreadManager::FawkesThreadManager()
{
  initializer = NULL;
  finalizer   = NULL;
  threads.clear();
  waitcond_timedthreads = new WaitCondition();
  __interrupt_timed_thread_wait = false;
  __aspect_collector = new FawkesThreadManagerAspectCollector(this);
}


/** Destructor. */
FawkesThreadManager::~FawkesThreadManager()
{
  // stop all threads, we call finalize, and we run through it as long as there are
  // still running threads, after that, we force the thread's death.
  for (tit = threads.begin(); tit != threads.end(); ++tit) {
    (*tit).second.force_stop(finalizer);
  }
  untimed_threads.force_stop(finalizer);
  threads.clear();

  delete waitcond_timedthreads;
  delete __aspect_collector;
}


/** Set initializer/finalizer.
 * This method has to be called before any thread is added/removed.
 * @param initializer thread initializer
 * @param finalizer thread finalizer
 */
void
FawkesThreadManager::set_inifin(ThreadInitializer *initializer, ThreadFinalizer *finalizer)
{
  this->initializer = initializer;
  this->finalizer   = finalizer;
}


/** Remove the given thread from internal structures.
 * Thread is removed from the internal structures. If the thread has the
 * BlockedTimingAspect then the hook is added to the changed list.
 *
 * @param t thread to remove
 * @param changed list of changed hooks, appropriate hook is added if necessary
 */
void
FawkesThreadManager::internal_remove_thread(Thread *t)
{
  BlockedTimingAspect *timed_thread;

  if ( (timed_thread = dynamic_cast<BlockedTimingAspect *>(t)) != NULL ) {
    // find thread and remove
    BlockedTimingAspect::WakeupHook hook = timed_thread->blockedTimingAspectHook();
    if ( threads.find(hook) != threads.end() ) {
      threads[hook].remove_locked(t);
    }
  } else {
    untimed_threads.remove_locked(t);
  }
}


/** Add the given thread to internal structures.
 * Thread is added to the internal structures. If the thread has the
 * BlockedTimingAspect then the hook is added to the changed list.
 *
 * @param t thread to add
 * @param changed list of changed hooks, appropriate hook is added if necessary
 */
void
FawkesThreadManager::internal_add_thread(Thread *t)
{
  BlockedTimingAspect *timed_thread;
  if ( (timed_thread = dynamic_cast<BlockedTimingAspect *>(t)) != NULL ) {
    BlockedTimingAspect::WakeupHook hook = timed_thread->blockedTimingAspectHook();

    if ( threads.find(hook) == threads.end() ) {
      threads[hook].set_name("FawkesThreadManagerList Hook %i", hook);
      threads[hook].set_maintain_barrier(true);
    }
    threads[hook].push_back_locked(t);

    waitcond_timedthreads->wake_all();
  } else {
    untimed_threads.push_back_locked(t);
  }
}


/** Add threads.
 * Add the given threads to the thread manager. The threads are initialised
 * as appropriate and started. See the class documentation for supported
 * specialisations of threads and the performed initialisation steps.
 * If the thread initializer cannot initalize one or more threads no thread
 * is added. In this regard the operation is atomic, either all threads are
 * added or none.
 * @param tl thread list with threads to add
 * @exception CannotInitializeThreadException thrown if at least one of the
 * threads could not be initialised
 */
void
FawkesThreadManager::add_maybelocked(ThreadList &tl, bool lock)
{
  if ( ! (initializer && finalizer) ) {
    throw NullPointerException("FawkesThreadManager: initializer/finalizer not set");
  }

  if ( tl.sealed() ) {
    throw Exception("Not accepting new threads from list that is not fresh, "
		    "list '%s' already sealed", tl.name());
  }

  tl.lock();

  // Try to initialise all threads
  try {
    tl.init(initializer, finalizer);
  } catch (Exception &e) {
    tl.unlock();
    throw;
  }

  tl.seal();
  tl.start();

  // All thread initialized, now add threads to internal structure
  MutexLocker locker(threads.mutex(), lock);
  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    internal_add_thread(*i);
  }

  tl.unlock();
}


/** Add one thread.
 * Add the given thread to the thread manager. The thread is initialized
 * as appropriate and started. See the class documentation for supported
 * specialisations of threads and the performed initialisation steps.
 * If the thread initializer cannot initalize the thread it is not added.
 * @param thread thread to add
 * @param lock if true the environment is locked before adding the thread
 * @exception CannotInitializeThreadException thrown if at least the
 * thread could not be initialised
 */
void
FawkesThreadManager::add_maybelocked(Thread *thread, bool lock)
{
  if ( thread == NULL ) {
    throw NullPointerException("FawkesThreadMananger: cannot add NULL as thread");
  }

  if ( ! (initializer && finalizer) ) {
    throw NullPointerException("FawkesThreadManager: initializer/finalizer not set");
  }

  try {
    initializer->init(thread);
  } catch (CannotInitializeThreadException &e) {
    e.append("Adding thread in FawkesThreadManager failed");
    throw;
  }

  thread->start();
  MutexLocker locker(threads.mutex(), lock);
  internal_add_thread(thread);
}


/** Remove the given threads.
 * The thread manager tries to finalize and stop the threads and then removes the
 * threads from the internal structures.
 *
 * This may fail if at least one thread of the given list cannot be finalized, for
 * example if prepare_finalize() returns false or if the thread finalizer cannot
 * finalize the thread. In this case a CannotFinalizeThreadException is thrown.
 *
 * @param tl threads to remove.
 * @exception CannotFinalizeThreadException At least one thread cannot be safely
 * finalized
 * @exception ThreadListNotSealedException if the given thread lits tl is not
 * sealed the thread manager will refuse to remove it
 */
void
FawkesThreadManager::remove_maybelocked(ThreadList &tl, bool lock)
{
  if ( ! (initializer && finalizer) ) {
    throw NullPointerException("FawkesThreadManager: initializer/finalizer not set");
  }


  if ( ! tl.sealed() ) {
    throw ThreadListNotSealedException("(FawkesThreadManager) Cannot remove unsealed thread "
				       "list. Not accepting unsealed list '%s' for removal",
				       tl.name());
  }

  tl.lock();
  MutexLocker locker(threads.mutex(), lock);

  try {
    if ( ! tl.prepare_finalize(finalizer) ) {
      tl.cancel_finalize();
      tl.unlock();
      throw CannotFinalizeThreadException("One or more threads in list '%s' cannot be "
					  "finalized", tl.name());
    }
  } catch (CannotFinalizeThreadException &e) {
    tl.unlock();
    throw;
  } catch (Exception &e) {
    tl.unlock();
    e.append("One or more threads in list '%s' cannot be finalized", tl.name());
    throw CannotFinalizeThreadException(e);
  }

  tl.stop();
  tl.finalize(finalizer);

  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    internal_remove_thread(*i);
  }

  tl.unlock();
}


/** Remove the given thread.
 * The thread manager tries to finalize and stop the thread and then removes the
 * thread from the internal structures.
 *
 * This may fail if the thread cannot be finalized, for
 * example if prepare_finalize() returns false or if the thread finalizer cannot
 * finalize the thread. In this case a CannotFinalizeThreadException is thrown.
 *
 * @param thread thread to remove.
 * @exception CannotFinalizeThreadException At least one thread cannot be safely
 * finalized
 */
void
FawkesThreadManager::remove_maybelocked(Thread *thread, bool lock)
{
  if ( thread == NULL ) return;

  if ( ! (initializer && finalizer) ) {
    throw NullPointerException("FawkesThreadManager: initializer/finalizer not set");
  }

  MutexLocker locker(threads.mutex(), lock);
  try {
    if ( ! thread->prepare_finalize() ) {
      thread->cancel_finalize();
      throw CannotFinalizeThreadException("Thread '%s'cannot be finalized", thread->name());
    }
  } catch (CannotFinalizeThreadException &e) {
    e.append("FawkesThreadManager cannot stop thread '%s'", thread->name());
    thread->cancel_finalize();
    throw;
  }

  thread->cancel();
  thread->join();
  finalizer->finalize(thread);
  thread->finalize();

  internal_remove_thread(thread);
}




/** Force removal of the given threads.
 * The thread manager tries to finalize and stop the threads and then removes the
 * threads from the internal structures.
 *
 * This will succeed even if a thread of the given list cannot be finalized, for
 * example if prepare_finalize() returns false or if the thread finalizer cannot
 * finalize the thread.
 *
 * <b>Caution, using this function may damage your robot.</b>
 *
 * @param tl threads to remove.
 * @exception ThreadListNotSealedException if the given thread lits tl is not
 * sealed the thread manager will refuse to remove it
 * The threads are removed from thread manager control. The threads will be stopped
 * before they are removed (may cause unpredictable results otherwise).
 */
void
FawkesThreadManager::force_remove(ThreadList &tl)
{
  if ( ! tl.sealed() ) {
    throw ThreadListNotSealedException("Not accepting unsealed list '%s' for removal",
				       tl.name());
  }

  tl.lock();
  threads.mutex()->stopby();
  tl.force_stop(finalizer);

  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    internal_remove_thread(*i);
  }

  tl.unlock();
}


/** Force removal of the given thread.
 * The thread manager tries to finalize and stop the thread and then removes the
 * thread from the internal structures.
 *
 * This will succeed even if the thread cannot be finalized, for
 * example if prepare_finalize() returns false or if the thread finalizer cannot
 * finalize the thread.
 *
 * <b>Caution, using this function may damage your robot.</b>
 *
 * @param thread thread to remove.
 * @exception ThreadListNotSealedException if the given thread lits tl is not
 * sealed the thread manager will refuse to remove it
 * The threads are removed from thread manager control. The threads will be stopped
 * before they are removed (may cause unpredictable results otherwise).
 */
void
FawkesThreadManager::force_remove(fawkes::Thread *thread)
{
  MutexLocker lock(threads.mutex());
  try {
    thread->prepare_finalize();
  } catch (Exception &e) {
    // ignore
  }

  thread->cancel();
  thread->join();
  if (finalizer) finalizer->finalize(thread);
  thread->finalize();

  internal_remove_thread(thread);
}


void
FawkesThreadManager::wakeup_and_wait(BlockedTimingAspect::WakeupHook hook,
				     unsigned int timeout_usec)
{
  threads.lock();
  unsigned int timeout_sec = 0;
  if (timeout_usec > 1000000) {
    timeout_sec   = timeout_usec / 1000000;
    timeout_usec -= timeout_sec  * 1000000;
  }

  if ( threads.find(hook) != threads.end() ) {
    threads.unlock();
    threads[hook].wakeup_and_wait(timeout_sec, timeout_usec * 1000);
    threads.lock();
    if ( threads[hook].size() == 0 ) {
      threads.erase(hook);
    }
    threads.unlock();
  } else {
    threads.unlock();
  }
}


void
FawkesThreadManager::wakeup(BlockedTimingAspect::WakeupHook hook, Barrier *barrier)
{
  threads.lock();
  if ( threads.find(hook) != threads.end() ) {
    threads.unlock();
    if ( barrier ) {
      threads[hook].wakeup(barrier);
    } else {
      threads[hook].wakeup();
    }
    threads.lock();
    if ( threads[hook].size() == 0 ) {
      threads.erase(hook);
    }
    threads.unlock();
  } else {
    threads.unlock();
  }
}


void
FawkesThreadManager::try_recover(std::list<std::string> &recovered_threads)
{
  threads.lock();
  for (tit = threads.begin(); tit != threads.end(); ++tit) {
    tit->second.try_recover(recovered_threads);
  }
  threads.unlock();
}


bool
FawkesThreadManager::timed_threads_exist()
{
  return (threads.size() > 0);
}


void
FawkesThreadManager::wait_for_timed_threads()
{
  __interrupt_timed_thread_wait = false;
  waitcond_timedthreads->wait();
  if ( __interrupt_timed_thread_wait ) {
    __interrupt_timed_thread_wait = false;
    throw InterruptedException("Waiting for timed threads was interrupted");
  }
}

void
FawkesThreadManager::interrupt_timed_thread_wait()
{
  __interrupt_timed_thread_wait = true;
  waitcond_timedthreads->wake_all();  
}



/** Get a thread collector to be used for an aspect initializer.
 * @return thread collector instance to use for ThreadProducerAspect.
 */
ThreadCollector *
FawkesThreadManager::aspect_collector() const
{
  return __aspect_collector;
}
