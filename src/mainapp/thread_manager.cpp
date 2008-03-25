
/***************************************************************************
 *  thread_manager.cpp - Thread manager
 *
 *  Generated: Thu Nov  3 19:11:31 2006 (on train to Cologne)
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

#include <mainapp/thread_manager.h>
#include <core/threading/thread.h>
#include <core/threading/barrier.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/wait_condition.h>
#include <core/threading/thread_initializer.h>
#include <core/threading/thread_finalizer.h>
#include <core/exceptions/software.h>

#include <aspect/blocked_timing.h>

/** @class FawkesThreadManager mainapp/thread_manager.h
 * Thread Manager.
 * This class provides a manager for the threads. Threads are memorized by
 * their wakeup hook. When the thread manager is deleted, all threads are
 * appropriately cancelled, joined and deleted. Thus the thread manager
 * can be used for "garbage collection" of threads.
 *
 * The thread manager allows easy wakeup of threads of a given wakeup hook.
 * The thread manager keeps track of needed barriers and supplies them to
 * the thread during wakeup as appropriate.
 *
 * The thread manager needs a thread initializer. Each thread that is added
 * to the thread manager is initialized with this. The runtime type information
 * (RTTI) supplied by C++ can be used to initialize threads if appropriate
 * (if the thread has certain aspects that need special treatment).
 *
 * @author Tim Niemueller
 */

/** Constructor.
 */
FawkesThreadManager::FawkesThreadManager()
{
  initializer = NULL;
  finalizer   = NULL;
  threads.clear();
  barriers.clear();
  wait_for_timed = new WaitCondition();
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

  // delete all barriers
  for (std::map< BlockedTimingAspect::WakeupHook, Barrier * >::iterator bit = barriers.begin(); bit != barriers.end(); ++bit) {
    delete (*bit).second;
  }
  barriers.clear();
  delete wait_for_timed;
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


/** Update barriers.
 * This sorts and uniques the given list and then updates all barriers for the changed
 * hooks that have as indicated by the changed list.
 * @param changed list of changed hooks
 */
void
FawkesThreadManager::update_barrier(BlockedTimingAspect::WakeupHook hook)
{
  if ( barriers.find(hook) != barriers.end() ) {
    delete barriers[hook];
    if ( threads.find(hook) == threads.end() ) {
      barriers.erase(hook);
    }
  }

  if ( threads.find(hook) != threads.end() ) {
    barriers[hook] = new Barrier(threads[hook].size() + 1);
  }
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
    for (ThreadList::iterator j = threads[hook].begin(); j != threads[hook].end(); ++j) {
      if ( *j == t ) {
	threads[ hook ].erase( j );
	break;
      }
    }
    if ( threads[hook].size() == 0 ) {
      threads.erase(hook);
    }
    update_barrier(hook);
  } else {
    untimed_threads.lock();
    for (ThreadList::iterator j = untimed_threads.begin(); j != untimed_threads.end(); ++j) {
      if ( *j == t ) {
	untimed_threads.erase( j );
	break;
      }
    }
    untimed_threads.unlock();
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
    threads[hook].lock();
    threads[hook].push_back(t);

    // Re-create barriers where necessary
    update_barrier(hook);

    threads[hook].unlock();
    wait_for_timed->wake_all();
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
FawkesThreadManager::add(ThreadList &tl)
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
    e.append("ThreadManager cannot start one or more threads of thread list '%s'",
	     tl.name());
    throw;
  }

  tl.seal();
  tl.start();

  // All thread initialized, now add threads to internal structure
  MutexLocker lock(threads.mutex());
  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    internal_add_thread(*i);
  }

  tl.unlock();
}


/** Add one thread.
 * Add the given thread to the thread manager. The threadis initialised
 * as appropriate and started. See the class documentation for supported
 * specialisations of threads and the performed initialisation steps.
 * If the thread initializer cannot initalize the thread it is not added.
 * @param thread thread to add
 * @exception CannotInitializeThreadException thrown if at least the
 * thread could not be initialised
 */
void
FawkesThreadManager::add(Thread *thread)
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
  MutexLocker lock(threads.mutex());
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
FawkesThreadManager::remove(ThreadList &tl)
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
  MutexLocker lock(threads.mutex());

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
FawkesThreadManager::remove(Thread *thread)
{
  if ( thread == NULL ) return;

  if ( ! (initializer && finalizer) ) {
    throw NullPointerException("FawkesThreadManager: initializer/finalizer not set");
  }

  MutexLocker lock(threads.mutex());
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
  MutexLocker lock(threads.mutex());
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
FawkesThreadManager::force_remove(Thread *thread)
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


/** Wakeup thread for given hook and wait for completion.
 * This will wakeup all threads registered for the given hook. Afterwards
 * this method will block until all threads finished their loop.
 * @param hook hook for which to wait for
 */
void
FawkesThreadManager::wakeup_and_wait(BlockedTimingAspect::WakeupHook hook)
{
  MutexLocker lock(threads.mutex());
  if ( threads.find(hook) != threads.end() ) {
    threads[hook].lock();
    threads[hook].wakeup_unlocked(barriers[hook]);
    if ( barriers.find(hook) != barriers.end() ) {
      barriers[hook]->wait();
    }
    threads[hook].unlock();
  }
}


/** Check if any timed threads exist.
 * @return true if threads exist that need to be woken up for execution, false
 * otherwise
 */
bool
FawkesThreadManager::timed_threads_exist() const
{
  return (threads.size() > 0);
}


/** Wait for timed threads.
 * Use this method to wait until a timed that is added to the thread manager.
 */
void
FawkesThreadManager::wait_for_timed_threads()
{
  wait_for_timed->wait();
}
