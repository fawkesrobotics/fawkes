
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
  pthread_mutex_t mutex;
};
/// @endcond


/** @class WaitCondition core/threading/wait_condition.h
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
 */


/** Constructor */
WaitCondition::WaitCondition()
{
  cond_data = new WaitConditionData();
  pthread_cond_init( &(cond_data->cond), NULL );
  pthread_mutex_init( &(cond_data->mutex), NULL );
}


/** Destructor */
WaitCondition::~WaitCondition()
{
  pthread_cond_destroy( &(cond_data->cond) );
  pthread_mutex_destroy( &(cond_data->mutex) );
  delete cond_data;
  cond_data = NULL;
}


/** Wait for the condition forever with internal mutex.
 * This waits forever until a wakup signal is received by another thread calling
 * wake_all() or wake_one(). An internal mutex is used that is locked immediately
 * before the wait starts and unlocked immediately after waiting is finished.
 */
void
WaitCondition::wait()
{
  Mutex mutex;
  mutex.lock();
  int err = pthread_cond_wait( &(cond_data->cond), &(mutex.mutex_data->mutex) );
  mutex.unlock();
  if ( err != 0 ) {
    throw Exception(err, "Waiting for wait condition failed");
  }
}

/** Wait for the condition forever with supplied mutex.
 * This waits forever until a wakup signal is received by another thread calling
 * wake_all() or wake_one(). The passed mutex must be locked or the result is
 * undefined. After the method returns the mutex is locked again.
 * @param mutex Mutex to wait on
 */
void
WaitCondition::wait(Mutex *mutex)
{
  int err = pthread_cond_wait( &(cond_data->cond), &(mutex->mutex_data->mutex) );
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
  err = pthread_cond_timedwait( &(cond_data->cond), &(mutex->mutex_data->mutex), &ts );

  if ( err == ETIMEDOUT ) {
    return false;
  } else if ( err != 0 ) {
    // some other error happened, a "real" error
    throw Exception(err, "Waiting for wait condition failed");
  } else {
    return true;
  }
}

/** Wait with absolute timeout on internal mutex.
 * This is a convenience method for abstimed_wait() mentioned above. Creates a
 * new mutex and calls the other function with this mutex.
 * @param sec Seconds of absolute time since the epoch (value compatible to
 * timeval tv_sec part is sufficient).
 * @param nanosec Nanoseconds part of the absolute timeout. Added to the seconds
 * part.
 * @return true, if the thread was woken up by another thread calling
 * wake_one() or wake_all(), false otherwise if the timeout has been reached
 * @exception Exception thrown if another error occurs for the POSIX wait condition
 */
bool
WaitCondition::abstimed_wait(long int sec, long int nanosec)
{
  Mutex mutex;
  return abstimed_wait(&mutex, sec, nanosec);
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
    if (ns > 1000000000) {
      s  += 1;
      ns -= 1000000000;
    }

    struct timespec ts = { s, ns };
    err = pthread_cond_timedwait( &(cond_data->cond), &(mutex->mutex_data->mutex), &ts );

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

/** Wait with relative timeout on internal mutex.
 * This is a convenience method for reltimed_wait() mentioned above. Creates a
 * new mutex and calls the other function with this mutex.
 * A timeout of (0,0) will cause this method to wait forever, similar to wait().
 * @param sec Number of seconds to wait
 * @param nanosec Number of nanoseconds to wait, added to seconds value
 * @return true, if the thread was woken up by another thread calling
 * wake_one() or wake_all(), false otherwise if the timeout has been reached
 * @exception Exception thrown if another error occurs for the POSIX wait condition
 */
bool
WaitCondition::reltimed_wait(unsigned int sec, unsigned int nanosec)
{
  if ( ! (sec || nanosec) ) {
    wait();
    return true;
  } else {
    Mutex mutex;
    return reltimed_wait(&mutex, sec, nanosec);
  }
}


/** Wake another thread waiting for this condition.
 * This wakes up any thread waiting for the condition, not a particular one. No guarantee
 * is given about the order of the woken up threads.
 */
void
WaitCondition::wake_one()
{
  pthread_mutex_lock( &(cond_data->mutex) );
  pthread_cond_signal( &(cond_data->cond) );
  pthread_mutex_unlock( &(cond_data->mutex) );
}


/** Wake up all waiting threads.
 * This wakes up all threads waiting for this condition.
 */
void
WaitCondition::wake_all()
{
  pthread_mutex_lock( &(cond_data->mutex) );
  pthread_cond_broadcast( &(cond_data->cond) );
  pthread_mutex_unlock( &(cond_data->mutex) );
}


} // end namespace fawkes
