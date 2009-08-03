
/***************************************************************************
 *  executor.h - Interface to exec BlockedTimingAspect threads
 *
 *  Created: Sat Aug 02 13:03:29 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <aspect/blocked_timing/executor.h>

namespace fawkes {

/** @class BlockedTimingExecutor <aspect/blocked_timing/executor.h>
 * Blocked timing executor.
 * This interface defines access to threads with the blocked timing aspect.
 * @author Tim Niemueller
 *
 * @fn void BlockedTimingExecutor::wakeup_and_wait(BlockedTimingAspect::WakeupHook hook, unsigned int timeout_usec)
 * Wakeup thread for given hook and wait for completion.
 * This will wakeup all threads registered for the given hook. Afterwards
 * this method will block until all threads finished their loop.
 * @param hook hook for which to wait for
 * @param timeout_usec timeout for thread execution in usec. 0 disables the
 * timeout and makes this function to wait forever.
 * @exception Exception thrown if the timeout was exceeded (at least one of the
 * threads for the given hook took longer than the desired time.
 *
 * @fn void BlockedTimingExecutor::wakeup(BlockedTimingAspect::WakeupHook hook, Barrier *barrier = 0)
 * Wakeup thread for given hook.
 * This will wakeup all threads registered for the given hook. With the optional
 * barrier you can synchronize to the end of the loop execution of the threads
 * of the given hook.
 * @param hook hook for which to wait for
 * @param barrier optional barrier that can be used to synchronize to the
 * end of the loop execution of the threads.
 *
 * @fn void BlockedTimingExecutor::try_recover(std::list<std::string> &recovered_threads)
 * Try to recover threads.
 * An advanced BlockedTimingExecutor might be able to detect deadlocked threads.
 * In this case this function should be called regularly to allow for recovering
 * threads that are back to normal operation.
 * @param recovered_threads upon return the names of any threads that could be
 * recovered from a bad state have been added to the list.
 *
 * @fn bool BlockedTimingExecutor::timed_threads_exist()
 * Check if any timed threads exist.
 * @return true if threads exist that need to be woken up for execution, false
 * otherwise
 *
 * @fn void BlockedTimingExecutor::wait_for_timed_threads()
 * Wait for timed threads.
 * Use this method to wait until a timed that is added to the thread manager.
 * Note that an InterruptedException is thrown if interrup_timed_thread_wait() is
 * called from another thread. You should catch this exception and stop the loop
 * execution.
 * @exception InterruptedException thrown if another thread calls
 * interrupt_timed_thread_wait()
 *
 * @fn void BlockedTimingExecutor::interrupt_timed_thread_wait()
 * Interrupt any currently running wait_for_timed_threads() and cause it to
 * throw an InterruptedException.
 *
 */

/** Virtual empty destructor. */
BlockedTimingExecutor::~BlockedTimingExecutor()
{
}

} // end namespace fawkes
