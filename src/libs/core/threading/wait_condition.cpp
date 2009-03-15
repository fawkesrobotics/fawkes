
/***************************************************************************
 *  wait_condition.cpp - condition variable implementation
 *
 *  Created: Thu Sep 14 21:43:30 2006
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/wait_condition.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_data.h>
#include <core/exception.h>

#include <pthread.h>
#include <cerrno>

namespace fawkes {

/// @cond INTERNALS
class WaitConditionData
{
 public:
  pthread_cond_t cond;
};
/// @endcond


/** @class WaitCondition <core/threading/wait_condition.h>
 * Wait until a given condition holds.
 * Consider two values x and y and you want to wait until they are equal.
 * For instance there may be a thread counting up after he has finished one
 * particular job before he goes to handle the next one. After 10 threads you
 * want to send out the produced entities in one batch run. So the sending
 * thread has to wait for the producing thread until 10 packages have been
 * produced. Simplified this could be implemented as
 *
 * @code
 * virtual void run()
 * {
 *   forever {
 *     mutex->lock();
 *     while (count != 10) {
 *       wait_condition->wait(mutex);
 *     }
 *   }
 * }
 * @endcode
 *
 * The other thread will wake up this waiting thread after each produced
 * package (the thread does not have to know after how many packages they are
 * sent out). The code could look like this:
 *
 * @code
 * virtual void run()
 * {
 *   forever {
 *     produce_package();
 *     wait_condition->wake_one();
 *   }
 * }
 * @endcode
 *
 * @ingroup Threading
 * @ingroup FCL
 * @see example_waitcond_serialize.cpp
 *
 * @author Tim Niemueller
 *
 * @fn bool WaitCondition::abstimed_wait(long int sec, long int nanosec)
 * Wait with absolute timeout on internal mutex.
 * This is a convenience method for
 * abstimed_wait(Mutex *mutex, long int sec, long int nanosec) supplying an
 * internal mutex as mutex.
 * @param sec Seconds of absolute time since the epoch (value compatible to
 * timeval tv_sec part is sufficient).
 * @param nanosec Nanoseconds part of the absolute timeout. Added to the seconds
 * part.
 * @return true, if the thread was woken up by another thread calling
 * wake_one() or wake_all(), false otherwise if the timeout has been reached
 * @exception Exception thrown if another error occurs for the POSIX wait condition
 *
 * @fn bool WaitCondition::reltimed_wait(unsigned int sec, unsigned int nanosec)
 * Wait with relative timeout on internal mutex.
 * This is a convenience method for
 * reltimed_wait(Mutex *mutex, long int sec, unsigned int nanosec) supplying an
 * internal mutex as mutex.
 * @param sec Number of seconds to wait
 * @param nanosec Number of nanoseconds to wait, added to seconds value
 * @return true, if the thread was woken up by another thread calling
 * wake_one() or wake_all(), false otherwise if the timeout has been reached
 * @exception Exception thrown if another error occurs for the POSIX wait condition
 *
 * @fn WaitCondition::wait()
 * Wait forever on internal mutex.
 * This is a convenience method for wait(Mutex *mutex).
 */


/** Constructor */
WaitCondition::WaitCondition()
{
  __cond_data = new WaitConditionData();
  __mutex     = new Mutex();
  pthread_cond_init( &(__cond_data->cond), NULL);
  __waiters = 0;
  __active_mutex = NULL;
}


/** Destructor */
WaitCondition::~WaitCondition()
{
  pthread_cond_destroy( &(__cond_data->cond) );
  delete __cond_data;
  delete __mutex;
}


/** Wait for the condition forever with internal mutex.
 * This waits forever until a wakup signal is received by another thread calling
 * wake_all() or wake_one(). An internal mutex is used that is locked immediately
 * before the wait starts and unlocked immediately after waiting is finished.
 */

/** Wait for the condition forever with supplied mutex.
 * This waits forever until a wakup signal is received by another thread calling
 * wake_all() or wake_one(). The passed mutex must be locked or the result is
 * undefined. After the method returns the mutex is locked again.
 * @param mutex Mutex to wait on
 */
void
WaitCondition::wait(Mutex *mutex)
{
  if ( __active_mutex && (__active_mutex != mutex) ) {
    throw Exception("WaitCondition is being used with another Mutex.");
  }
  ++__waiters; __active_mutex = mutex;
  int err = pthread_cond_wait( &(__cond_data->cond), &(mutex->mutex_data->mutex) );
  if (--__waiters == 0)  __active_mutex = NULL;
  if ( err != 0 ) {
    throw Exception(err, "Waiting for wait condition failed");
  }
}


/** Wait with absolute timeout on supplied mutex.
 * This waits for the given mutex until either a wakup signal is received or
 * the timeout has passed. The timeout has to be given in absolute system time,
 * a simulated clock source cannot be used.
 * @param mutex Mutex to wait on
 * @param sec Seconds of absolute time since the epoch (value compatible to
 * timeval tv_sec part is sufficient).
 * @param nanosec Nanoseconds part of the absolute timeout. Added to the seconds
 * part.
 * @return true, if the thread was woken up by another thread calling
 * wake_one() or wake_all(), false otherwise if the timeout has been reached
 * @exception Exception thrown if another error occurs for the POSIX wait condition
 */
bool
WaitCondition::abstimed_wait(Mutex *mutex, long int sec, long int nanosec)
{
  int err = 0;
  struct timespec ts = { sec, nanosec };

  if ( __active_mutex && (__active_mutex != mutex) ) {
    throw Exception("WaitCondition is being used with another Mutex.");
  }
  ++__waiters; __active_mutex = mutex;
  err = pthread_cond_timedwait( &(__cond_data->cond), &(mutex->mutex_data->mutex), &ts );
  if (--__waiters == 0)  __active_mutex = NULL;

  if ( err == ETIMEDOUT ) {
    return false;
  } else if ( err != 0 ) {
    // some other error happened, a "real" error
    throw Exception(err, "Waiting for wait condition failed");
  } else {
    return true;
  }
}

/** Wait with relative timeout on supplied mutex.
 * This waits for the given mutex until either a wakup signal is received or
 * the timeout has passed. The timeout has to be given in relative system time.
 * It is added to the current time and is then used similar to abstime_wait().
 * A timeout of (0,0) will cause this method to wait forever, similar to wait().
 * @param mutex Mutex to wait on
 * @param sec Number of seconds to wait
 * @param nanosec Number of nanoseconds to wait, added to seconds value
 * @return true, if the thread was woken up by another thread calling
 * wake_one() or wake_all(), false otherwise if the timeout has been reached
 * @exception Exception thrown if another error occurs for the POSIX wait condition
 */
bool
WaitCondition::reltimed_wait(Mutex *mutex, unsigned int sec, unsigned int nanosec)
{
  if ( ! (sec || nanosec) ) {
    wait(mutex);
    return true;
  } else {
    long err = 0;
    struct timespec now;
    if ( clock_gettime(CLOCK_REALTIME, &now) != 0 ) {
      throw Exception(err, "WaitCondition::reltimed_wait: Failed to get current time");
    }

    long int s  = now.tv_sec  + sec;
    long int ns = now.tv_nsec + nanosec;
    if (ns >= 1000000000) {
      s  += 1;
      ns -= 1000000000;
    }

    struct timespec ts = { s, ns };
    if ( __active_mutex && (__active_mutex != mutex) ) {
      throw Exception("WaitCondition is being used with another Mutex.");
    }
    ++__waiters; __active_mutex = mutex;
    err = pthread_cond_timedwait( &(__cond_data->cond), &(mutex->mutex_data->mutex), &ts );
    if (--__waiters == 0)  __active_mutex = NULL;

    if ( err == ETIMEDOUT ) {
      return false;
    } else if ( err != 0 ) {
      // some other error happened, a "real" error
      throw Exception(err, "Waiting for wait condition failed");
    } else {
      return true;
    }
  }
}


/** Wake another thread waiting for this condition.
 * This wakes up any thread waiting for the condition, not a particular one. No guarantee
 * is given about the order of the woken up threads.
 */
void
WaitCondition::wake_one()
{
  if ( ! __active_mutex )  return;
  bool unlock = __active_mutex->try_lock();
  pthread_cond_signal( &(__cond_data->cond) );
  if (unlock) __active_mutex->unlock();
}


/** Wake up all waiting threads.
 * This wakes up all threads waiting for this condition.
 */
void
WaitCondition::wake_all()
{
  if ( ! __active_mutex )  return;
  bool unlock = __active_mutex->try_lock();
  pthread_cond_broadcast( &(__cond_data->cond) );
  if (unlock)  __active_mutex->unlock();
}


} // end namespace fawkes
