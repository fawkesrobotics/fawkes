
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
#include <core/threading/thread_initializer.h>
#include <core/threading/thread_finalizer.h>

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
 * @param initializer thread initializer
 * @param finalizer thread finalizer
 */
FawkesThreadManager::FawkesThreadManager(ThreadInitializer *initializer,
					 ThreadFinalizer *finalizer)
{
  this->initializer = initializer;
  this->finalizer   = finalizer;
  threads.clear();
  barriers.clear();
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
}



/** Update barriers.
 * This sorts and uniques the given list and then updates all barriers for the changed
 * hooks that have as indicated by the changed list.
 * @param changed list of changed hooks
 */
void
FawkesThreadManager::update_barriers(std::list<BlockedTimingAspect::WakeupHook> &changed)
{
  changed.sort();
  changed.unique();
  for (std::list<BlockedTimingAspect::WakeupHook>::iterator i = changed.begin(); i != changed.end(); ++i) {
    if ( barriers.find(*i) != barriers.end() ) {
      delete barriers[*i];
      if ( threads.find(*i) == threads.end() ) {
	barriers.erase(*i);
      }
    }

    if ( threads.find(*i) != threads.end() ) {
      barriers[*i] = new Barrier(threads[*i].size() + 1);
    }
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
FawkesThreadManager::internal_remove_thread(Thread *t,
					    std::list<BlockedTimingAspect::WakeupHook> &changed)
{
  BlockedTimingAspect *timed_thread;

  if ( (timed_thread = dynamic_cast<BlockedTimingAspect *>(t)) != NULL ) {
    // find thread and remove
    BlockedTimingAspect::WakeupHook hook = timed_thread->blockedTimingAspectHook();
    threads[hook].lock();
    for (ThreadList::iterator j = threads[hook].begin(); j != threads[hook].end(); ++j) {
      if ( *j == t ) {
	threads[ hook ].erase( j );
	break;
      }
    }
    if ( threads[hook].size() == 0 ) {
      threads.erase(hook);
    }
    changed.push_back(hook);
    threads[hook].unlock();
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
FawkesThreadManager::internal_add_thread(Thread *t,
					 std::list<BlockedTimingAspect::WakeupHook> &changed)
{
  BlockedTimingAspect *timed_thread;
  if ( (timed_thread = dynamic_cast<BlockedTimingAspect *>(t)) != NULL ) {
    threads[ timed_thread->blockedTimingAspectHook() ].lock();
    threads[ timed_thread->blockedTimingAspectHook() ].push_back(t);
    changed.push_back(timed_thread->blockedTimingAspectHook());
    threads[ timed_thread->blockedTimingAspectHook() ].unlock();
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
  std::list<BlockedTimingAspect::WakeupHook> changed;

  if ( tl.sealed() ) {
    Exception e("Not accepting new threads from list that is not fresh");
    e.append("Threads in list '%s' already sealed", tl.name());
  }

  tl.lock();
  changed.clear();

  // Try to initialise all threads
  try {
    tl.init(initializer);
  } catch (Exception &e) {
    tl.unlock();
    e.append("ThreadManager cannot start one or more threads of thread list '%s'",
	     tl.name());
    throw;
  }

  tl.seal();

  // All thread initialized, now add threads to internal structure
  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    internal_add_thread(*i, changed);
  }

  // Re-create barriers where necessary
  update_barriers(changed);

  tl.start();
  tl.unlock();
}


/** Add thread from list deferred.
 * This will start the initialization of the threads deferred.
 * The threads are not yet added to the internal structures.
 * @param tl thread list to add deferred
 */
void
FawkesThreadManager::add_deferred(ThreadList &tl)
{
  if ( tl.sealed() ) {
    Exception e("Not accepting new threads from list that is not fresh");
    e.append("Threads in list '%s' already sealed", tl.name());
  }

  tl.lock();

  tl.seal();
  tl.init_deferred(initializer);
}


/** Check if deferred add is done.
 * @param tl thread list to check
 * @return true if the deferred add is odone, false otherwise.
 */
bool
FawkesThreadManager::deferred_add_done(ThreadList &tl)
{
  std::list<BlockedTimingAspect::WakeupHook> changed;
  changed.clear();

  try {
    if ( tl.deferred_init_done() ) {
      // All thread initialized, now add threads to internal structure
      for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
	internal_add_thread(*i, changed);
      }

      // Re-create barriers where necessary
      update_barriers(changed);

      tl.start();
      tl.unlock();

      return true;
    } else {
      return false;
    }
  } catch (Exception &e) {
    tl.unlock();
    throw;
  }
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
  try {
    initializer->init(thread);
  } catch (CannotInitializeThreadException &e) {
    e.append("Adding thread in FawkesThreadManager failed");
    throw;
  }

  std::list<BlockedTimingAspect::WakeupHook> changed;
  internal_add_thread(thread, changed);
  update_barriers(changed);

  thread->start();
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
  std::list<BlockedTimingAspect::WakeupHook> changed;

  if ( ! tl.sealed() ) {
    ThreadListNotSealedException e("Cannot remove unsealed thread list");
    e.append("Not accepting unsealed list '%s' for removal", tl.name());
    throw e;
  }

  changed.clear();

  tl.lock();

  try {
    if ( ! tl.prepare_finalize(finalizer) ) {
      tl.cancel_finalize();
      tl.unlock();
      CannotFinalizeThreadException e("One or more threads cannot be finalized");
      e.append("One or more threads in list '%s' cannot be finalized", tl.name());
      throw e;
    }
  } catch (CannotFinalizeThreadException &e) {
    tl.unlock();
    throw;
  } catch (Exception &e) {
    tl.unlock();
    e.append("One or more threads in list '%s' cannot be finalized", tl.name());
    throw CannotFinalizeThreadException(e);
  }

  tl.finalize(finalizer);
  tl.stop();
  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    internal_remove_thread(*i, changed);
  }

  update_barriers(changed);

  tl.unlock();
}


/** Remove the given threads deferred.
 * The thread manager tries to finalize and stop the threads and then removes the
 * threads from the internal structures.
 *
 * The finalization is just started, but not necessarily finished (deferred
 * operation). Use deferred_remove_done() to check for the result.
 *
 * @param tl threads to remove.
 * @exception ThreadListNotSealedException if the given thread lits tl is not
 * sealed the thread manager will refuse to remove it
 */
void
FawkesThreadManager::remove_deferred(ThreadList &tl)
{
  if ( ! tl.sealed() ) {
    ThreadListNotSealedException e("Cannot remove unsealed thread list");
    e.append("Not accepting unsealed list '%s' for removal", tl.name());
    throw e;
  }

  tl.lock();
  tl.finalize_deferred(finalizer);
}


/** Check if deferred removal is done.
 * @param tl thread list to check
 * @return true if deferred removal is done, false otherwise
 */
bool
FawkesThreadManager::deferred_remove_done(ThreadList &tl)
{
  std::list<BlockedTimingAspect::WakeupHook> changed;
  changed.clear();

  try {
    if ( tl.deferred_finalize_done() ) {

      tl.stop();

      // All thread initialized, now add threads to internal structure
      for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
	internal_remove_thread(*i, changed);
      }

      // Re-create barriers where necessary
      update_barriers(changed);

      tl.unlock();

      return true;
    } else {
      return false;
    }
  } catch (Exception &e) {
    tl.unlock();
    throw;
  }
}


/** Remove the given threads.
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
  try {
    if ( ! thread->prepare_finalize() ) {
      thread->cancel_finalize();
      CannotFinalizeThreadException e("Thread cannot be finalized");
      e.append("Threads '%s' cannot be finalized", thread->name());
      throw e;
    }
  } catch (CannotFinalizeThreadException &e) {
    e.append("FawkesThreadManager cannot stop thread '%s'", thread->name());
    thread->cancel_finalize();
    throw;
  }
  thread->finalize();
  thread->cancel();
  thread->join();

  std::list<BlockedTimingAspect::WakeupHook> changed;
  internal_remove_thread(thread, changed);

  update_barriers(changed);
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
  std::list<BlockedTimingAspect::WakeupHook> changed;

  if ( ! tl.sealed() ) {
    ThreadListNotSealedException e("Thread list not sealed");
    e.append("Not accepting unsealed list '%s' for removal", tl.name());
    throw e;
  }

  changed.clear();

  tl.lock();
  tl.force_stop(finalizer);

  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    internal_remove_thread(*i, changed);
  }

  update_barriers(changed);

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
  try {
    thread->prepare_finalize();
  } catch (Exception &e) {
    // ignore
  }
  thread->finalize();
  thread->cancel();
  thread->join();

  std::list<BlockedTimingAspect::WakeupHook> changed;
  internal_remove_thread(thread, changed);

  update_barriers(changed);
}


/** Wakeup threads for given hook.
 * @param hook hook for which to wakup threads.
 */
void
FawkesThreadManager::wakeup(BlockedTimingAspect::WakeupHook hook)
{
  if ( threads.find(hook) != threads.end() ) {
    threads[hook].wakeup(barriers[hook]);
  }
}


/** Wait for threads for given hook to complete.
 * @param hook hook for which to wait for
 */
void
FawkesThreadManager::wait(BlockedTimingAspect::WakeupHook hook)
{
  if ( barriers.find(hook) != barriers.end() ) {
    barriers[hook]->wait();
  }
}
