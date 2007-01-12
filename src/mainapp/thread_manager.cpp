
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
 */
FawkesThreadManager::FawkesThreadManager(ThreadInitializer *initializer)
{
  this->initializer = initializer;
  threads.clear();
  barriers.clear();
}


/** Destructor. */
FawkesThreadManager::~FawkesThreadManager()
{
  // stop all threads
  for (tit = threads.begin(); tit != threads.end(); ++tit) {
    stop((*tit).second);
  }
  stop(untimed_threads);
  threads.clear();

  // delete all barriers
  for (std::map< BlockedTimingAspect::WakeupHook, Barrier * >::iterator bit = barriers.begin(); bit != barriers.end(); ++bit) {
    delete (*bit).second;
  }
  barriers.clear();
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
  BlockedTimingAspect *timed_thread;

  tl.lock();
  changed.clear();

  // Try to initialise all threads
  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    try {
      initializer->init(*i);
    } catch (CannotInitializeThreadException &e) {
      e.append("Adding thread in FawkesThreadManager failed");
      tl.unlock();
      throw;
    }
  }

  // All thread initialized, now add threads to internal structure
  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    if ( (timed_thread = dynamic_cast<BlockedTimingAspect *>(*i)) != NULL ) {
      threads[ timed_thread->blocked_timing_hook() ].lock();
      threads[ timed_thread->blocked_timing_hook() ].push_back(*i);
      changed.push_back(timed_thread->blocked_timing_hook());
      threads[ timed_thread->blocked_timing_hook() ].unlock();
    } else {
      untimed_threads.lock();
      untimed_threads.push_back(*i);
      untimed_threads.unlock();
    }
  }

  // Re-create barriers where necessary
  changed.sort();
  changed.unique();
  for (std::list<BlockedTimingAspect::WakeupHook>::iterator i = changed.begin(); i != changed.end(); ++i) {
    if ( barriers.find(*i) != barriers.end() ) {
      delete barriers[*i];
    }
    // +1 for manager thread that waits for the barrier
    barriers[*i] = new Barrier(threads[*i].size() + 1);
  }

  start(tl);
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
  BlockedTimingAspect *timed_thread;
  try {
    initializer->init(thread);
  } catch (CannotInitializeThreadException &e) {
    e.append("Adding thread in FawkesThreadManager failed");
    throw;
  }

  if ( (timed_thread = dynamic_cast<BlockedTimingAspect *>(thread)) != NULL ) {
    BlockedTimingAspect::WakeupHook hook = timed_thread->blocked_timing_hook();
    threads[hook].lock();
    threads[hook].push_back(thread);
    if ( barriers.find(hook) != barriers.end() ) {
      delete barriers[hook];
    }
    barriers[hook] = new Barrier(threads[hook].size() + 1);
    threads[hook].unlock();
  } else {
    untimed_threads.push_back(thread);
  }

  thread->start();
}


/** Remove the given threads.
 * The threads are removed from thread manager control. The threads will be stopped
 * before they are removed (may cause unpredictable results otherwise).
 * @param tl threads to remove.
 */
void
FawkesThreadManager::remove(ThreadList &tl)
{
  std::list<BlockedTimingAspect::WakeupHook> changed;
  BlockedTimingAspect *timed_thread;

  changed.clear();

  tl.lock();
  stop(tl);
  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    if ( (timed_thread = dynamic_cast<BlockedTimingAspect *>(*i)) != NULL ) {
      // find thread and remove
      BlockedTimingAspect::WakeupHook hook = timed_thread->blocked_timing_hook();
      threads[hook].lock();
      for (ThreadList::iterator j = threads[hook].begin(); j != threads[hook].end(); ++j) {
	if ( *j == *i ) {
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
	if ( *j == *i ) {
	  untimed_threads.erase( j );
	  break;
	}
      }
      untimed_threads.unlock();
    }
  }  
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
  tl.unlock();
}


/** Remove the given thread.
 * The thread is removed from thread manager control. The thread will be stopped
 * before it is removed (may cause unpredictable results otherwise).
 * @param thread thread to remove.
 */
void
FawkesThreadManager::remove(Thread *thread)
{
  BlockedTimingAspect *timed_thread;
  if ( (timed_thread = dynamic_cast<BlockedTimingAspect *>(thread)) != NULL ) {
    BlockedTimingAspect::WakeupHook hook = timed_thread->blocked_timing_hook();
    threads[hook].lock();
    for (ThreadList::iterator j = threads[hook].begin(); j != threads[hook].end(); ++j) {
      if ( *j == thread ) {
	threads[ hook ].erase( j );
	break;
      }
    }
    if ( threads[hook].size() == 0 ) {
      threads.erase(hook);
    }

    // Check if barrier needs to be re-initialized
    if ( barriers.find(hook) != barriers.end() ) {
      delete barriers[hook];
      if ( threads.find(hook) == threads.end() ) {
	barriers.erase(hook);
      }
    }

    if ( threads.find(hook) != threads.end() ) {
      barriers[hook] = new Barrier(threads[hook].size() + 1);
    }

    threads[hook].unlock();
  } else {
    untimed_threads.lock();
    for (ThreadList::iterator j = untimed_threads.begin(); j != untimed_threads.end(); ++j) {
      if ( *j == thread ) {
	untimed_threads.erase( j );
	break;
      }
    }
    untimed_threads.unlock();
  }

  thread->cancel();
  thread->join();
}

/** Start the given threads.
 * The threads are started, no matter whether they have been added or not.
 * @param tl thread list of threads to start
 */
void
FawkesThreadManager::start(ThreadList &tl)
{
  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    (*i)->start();
  }
}


/** Stop threads.
 * The threads are stopped, no matter whether they have been added or not.
 * @param tl thread list of threads to stop
 */
void
FawkesThreadManager::stop(ThreadList &tl)
{
  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    (*i)->cancel();
    (*i)->join();
  }
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
