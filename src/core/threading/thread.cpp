
/***************************************************************************
 *  thread.cpp - implementation of threads, based on pthreads
 *
 *  Generated: Thu Sep 14 13:26:39 2006
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

#include <core/threading/thread.h>

#include <pthread.h>

/** @def forever
 * Shortcut for "while (1)".
 * @relates Thread
 */

/** @class Thread core/threading/thread.h
 * Thread class encapsulation of pthreads.
 * This is the base class for all threads in Fawkes. Derive this class for
 * your thread.
 *
 * There are two major ways to implement threads. The recommended way is to
 * implement loop(). The default run() implementation will call loop()
 * continuously. An implicit cancel point is set after each loop.
 *
 * If you need a more complex behaviour you may also override run() and
 * implement your own thread behavior.
 * That that without taking special care the advanced debug functionality
 * will not available for threads.
 *
 * @ingroup Threading
 * @ingroup FCL
 * @see loop()
 * @see run()
 * @see example_barrier.cpp
 * @see example_mutex_count.cpp
 * @see example_rwlock.cpp
 * @see example_waitcond_serialize.cpp
 *
 * @author Tim Niemueller
 */


/** Virtual empty destructor. */
Thread::~Thread()
{
}


/** Call this method to actuall start.
 * This method has to be called after the thread has been instantiated and
 * initialized to startup.
 * @return true, if the thread started successfully, false otherwise. error()
 * will return the error value in that case
 */
bool
Thread::start()
{
  int err;
  if ( (err = pthread_create(&thread_id, NULL, Thread::entry, this)) != 0) {
    // An error occured
    return false;
  }

  return true;
}


/** Entry point for the thread.
 * This is an utility method that acts as an entry point to the thread.
 * It is called automatically when you start the thread and will call run()
 * @param pthis a pointer to the instance that triggered the run of this method
 */
/* static */  void *
Thread::entry(void *pthis)
{
  Thread *t = (Thread *)pthis;
  t->run();
  return NULL;
}


/** Exit the thread.
 * You may call this from within your run() method to exit the thread.
 * @see run()
 */
void
Thread::exit()
{
  pthread_exit(NULL);
}


/** Join the thread.
 * This waites for the thread to exit.
 */
void
Thread::join()
{
  void *dont_care;
  pthread_join(thread_id, &dont_care);
}


/** Detach the thread.
 * Memory claimed by the thread will be automatically freed after the
 * thread exits. You can no longer join this thread.
 */
void
Thread::detach()
{
  pthread_detach(thread_id);
}


/** Cancel a thread.
 * Use this to cancel the thread.
 */
void
Thread::cancel()
{
  pthread_cancel(thread_id);
}


/** Set cancellation point.
 * Tests if the thread has been canceled and if so exits the thread.
 */
void
Thread::test_cancel()
{
  pthread_testcancel();
}


/** Check if two threads are the same.
 * @param thread Thread to compare this thread to.
 * @return true, if the threads are equal, false otherwise.
 */
bool
Thread::operator==(const Thread &thread)
{
  return ( pthread_equal(thread_id, thread.thread_id) != 0 );
}


/** Code to execute in the thread.
 * Executes loop() in each cycle. This is the default implementation and if
 * you need a more specific behaviour you can override this run() method and
 * ignore loop().
 */
void
Thread::run()
{
  forever {
    loop();
    test_cancel();
  }
}


/** Code to execute in the thread.
 * Implement this method to hold the code you want to be executed continously.
 */
void
Thread::loop()
{
}
