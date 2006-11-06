
/***************************************************************************
 *  thread_manager.cpp - Fawkes thread manager
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
#include <blackboard/main_thread.h>
#include <blackboard/interface_manager.h>
#include <core/threading/barrier.h>
#include <blackboard/bbthread.h>

/** @class NoInterfaceManagerException mainapp/thread_manager.h
 * This exception is thrown if no interface manager has been set.
 * @ingroup Exceptions
 * @fn NoInterfaceManagerException::NoInterfaceManagerException()
 * Constructor.
 */

/** @class InvalidWakeupHookException mainapp/thread_manager.h
 * This exception is thrown if it is tried to wakeup threads with hook
 * WAKEUP_HOOK_NONE.
 * @ingroup Exceptions
 * @fn InvalidWakeupHookException::InvalidWakeupHookException()
 * Constructor.
 */

/** @class FawkesThreadManager mainapp/thread_manager.h
 * Fawkes Thread Manager.
 * This class provides a manager for the threads running in Fawkes. Threads
 * from from plugins are added and removed as appropriate. They are also
 * started by the thread manager. The thread manager also keeps track of
 * needed barriers and supplies them to the thread during wakeup as
 * appropriate. Some special thread types are recognized and appropriately
 * initialized. These types are:
 * - FawkesThread
 *   Fawkes threads are queried for the desired hook and called as appropriate
 *   in the main loop.
 * - BlackBoardThread
 *   The interface manager is set for this type of threads. It is guaranteed
 *   that this happens before the thread is started.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
FawkesThreadManager::FawkesThreadManager()
{
  threads.clear();
  barriers.clear();
  interface_manager = NULL;
}


/** Destructor. */
FawkesThreadManager::~FawkesThreadManager()
{
  // stop all threads
  for (tit = threads.begin(); tit != threads.end(); ++tit) {
    stop((*tit).second);
  }
  threads.clear();

  // delete all barriers
  for (std::map< FawkesThread::WakeupHook, Barrier * >::iterator bit = barriers.begin(); bit != barriers.end(); ++bit) {
    delete (*bit).second;
  }
  barriers.clear();
}


/** Add threads.
 * Add the given threads to the thread manager. The threads are initialised
 * as appropriate and started. See the class documentation for supported
 * specialisations of threads and the performed initialisation steps.
 * @param tl thread list with threads to add
 */
void
FawkesThreadManager::add(ThreadList &tl)
{
  std::list<FawkesThread::WakeupHook> changed;
  BlackBoardMainThread *bb_main;
  FawkesThread *fawkes_thread;
  BlackBoardThread *bb_thread;

  changed.clear();

  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    if ( (bb_main = dynamic_cast<BlackBoardMainThread *>(*i)) != NULL ) {
      // we found the main thread!
      interface_manager = bb_main->getInterfaceManager();
    }
    if ( (bb_thread = dynamic_cast<BlackBoardThread *>(*i)) != NULL ) {
      if ( interface_manager == NULL ) {
	throw NoInterfaceManagerException();
      }
      bb_thread->setInterfaceManager( interface_manager );
    }
    if ( (fawkes_thread = dynamic_cast<FawkesThread *>(*i)) != NULL ) {
      threads[ fawkes_thread->hook() ].push_back(fawkes_thread);
      changed.push_back(fawkes_thread->hook());
    } else {
      threads[ FawkesThread::WAKEUP_HOOK_NONE ].push_back(*i);
    }
  }
  changed.sort();
  changed.unique();
  for (std::list<FawkesThread::WakeupHook>::iterator i = changed.begin(); i != changed.end(); ++i) {
    if ( barriers.find(*i) != barriers.end() ) {
      delete barriers[*i];
    }
    // +1 for manager thread that waits for the barrier
    barriers[*i] = new Barrier(threads[*i].size() + 1);
  }

  start(tl);
}


/** Remove the given threads.
 * The threads are removed from thread manager control. The threads will be stopped
 * before they are removed (may cause unpredictable results otherwise).
 * @param tl threads to remove.
 */
void
FawkesThreadManager::remove(ThreadList &tl)
{
  std::list<FawkesThread::WakeupHook> changed;
  FawkesThread *bb_thread;

  changed.clear();

  stop(tl);
  tl.lock();
  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    if ( (bb_thread = dynamic_cast<FawkesThread *>(*i)) != NULL ) {
      // find thread and remove
      FawkesThread::WakeupHook hook = bb_thread->hook();
      threads[hook].lock();
      for (ThreadList::iterator j = threads[hook].begin(); j != threads[hook].end(); ++j) {
	if ( *j == bb_thread ) {
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
      threads[ FawkesThread::WAKEUP_HOOK_NONE ].lock();
      threads[ FawkesThread::WAKEUP_HOOK_NONE ].push_back(*i);
      threads[ FawkesThread::WAKEUP_HOOK_NONE ].unlock();
    }
  }  
  changed.sort();
  changed.unique();
  for (std::list<FawkesThread::WakeupHook>::iterator i = changed.begin(); i != changed.end(); ++i) {
    if ( barriers.find(*i) != barriers.end() ) {
      delete barriers[*i];
      if ( threads.find(*i) == threads.end() ) {
	barriers.erase(*i);
      }
    }

    if ( threads.find(*i) != threads.end() ) {
      barriers[*i] = new Barrier(threads[*i].size());
    }
  }
}


/** Start the given threads.
 * The threads are started, no matter whether they have been added or not.
 * @param tl thread list of threads to start
 */
void
FawkesThreadManager::start(ThreadList &tl)
{
  tl.lock();
  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    (*i)->start();
  }
  tl.unlock();
}


/** Stop threads.
 * The threads are stopped, no matter whether they have been added or not.
 * @param tl thread list of threads to stop
 */
void
FawkesThreadManager::stop(ThreadList &tl)
{
  tl.lock();
  for (ThreadList::iterator i = tl.begin(); i != tl.end(); ++i) {
    (*i)->cancel();
    (*i)->join();
  }
  tl.unlock();
}


/** Wakeup threads for given hook.
 * @param hook hook for which to wakup threads.
 */
void
FawkesThreadManager::wakeup(FawkesThread::WakeupHook hook)
{
  if ( hook == FawkesThread::WAKEUP_HOOK_NONE ) {
    throw InvalidWakeupHookException();
  }
  if ( threads.find(hook) != threads.end() ) {
    threads[hook].wakeup(barriers[hook]);
  }
}


/** Wait for threads for given hook to complete.
 * @param hook hook for which to wait for
 */
void
FawkesThreadManager::wait(FawkesThread::WakeupHook hook)
{
  if ( hook == FawkesThread::WAKEUP_HOOK_NONE ) {
    throw InvalidWakeupHookException();
  }
  if ( barriers.find(hook) != barriers.end() ) {
    barriers[hook]->wait();
  }
}


/** Get interface manager.
 * @return current interface manager.
 */
BlackBoardInterfaceManager *
FawkesThreadManager::getInterfaceManager() const
{
  if ( interface_manager == NULL ) {
    throw NoInterfaceManagerException();
  }
  return interface_manager;
}
