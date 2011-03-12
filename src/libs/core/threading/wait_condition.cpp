
/***************************************************************************
 *  wait_condition.cpp - condition variable implementation
 *
 *  Created: Thu Sep 14 21:43:30 2006
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

#include <core/threading/wait_condition.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_data.h>
#include <core/exception.h>

#include <pthread.h>
#include <cerrno>
#if defined(__MACH__) && defined(__APPLE__)
#  include <sys/time.h>
#endif

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
 *       wait_condition->wait();
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
 * The WaitCondition can operate in two principal modes, either with an internal
 * or with an external Mutex. If no mutex is passed to the constructor an
 * internal mutex is created and used. If a mutex is passed this instance is used,
 * but ownership is not claimed and you have to delete it manually. Additionally,
 * for external mutexes they are <i>never</i> locked by the wait condition. For
 * external mutexes you get all the freedom, but also have the duty to ensure
 * proper locking from the outside! This applies to wait and wake methods.
 *
 * @ingroup Threading
 * @ingroup FCL
 * @see Mutex
 * @see qa_waitcond_serialize.cpp
 * @see qa_waitcond.cpp
 *
 * @author Tim Niemueller
 *
 */


/** Constructor.
 * @param mutex the mutex used for this wait condition. If none is given, an
 * internal mutex will be created and used.
 */
WaitCondition::WaitCondition(Mutex *mutex)
{
  __cond_data   = new WaitConditionData();
  pthread_cond_init( &(__cond_data->cond), NULL);
  if (mutex) {
    __mutex     = mutex;
    __own_mutex = false;
  } else {
    __mutex     = new Mutex();
    __own_mutex = true;
  }
}


/** Destructor. */
WaitCondition::~WaitCondition()
{
  pthread_cond_destroy( &(__cond_data->cond) );
  delete __cond_data;
  if (__own_mutex) {
    delete __mutex;
  }
}


/** Wait for the condition forever.
 * This waits forever until a wakup signal is received by another thread calling
 * wake_all() or wake_one(). If an external mutex is used it must be locked or
 * before calling wait() or the result is undefined. After the method returns
 * the mutex is locked again.
 */
void
WaitCondition::wait()
{
  int err;
  if ( __own_mutex) {
    __mutex->lock();
    err = pthread_cond_wait( &(__cond_data->cond), &(__mutex->mutex_data->mutex) );
    __mutex->unlock();
  } else {
    err = pthread_cond_wait( &(__cond_data->cond), &(__mutex->mutex_data->mutex) );
  }
  if ( err != 0 ) {
    throw Exception(err, "Waiting for wait condition failed");
  }
}


/** Wait with absolute timeout.
 * This waits for the given mutex until either a wakup signal is received or
 * the timeout has passed. The timeout has to be given in absolute system time,
 * a simulated clock source cannot be used.
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
  int err = 0;
  struct timespec ts = { sec, nanosec };

  if ( __own_mutex) {
    __mutex->lock();
    err = pthread_cond_timedwait( &(__cond_data->cond), &(__mutex->mutex_data->mutex), &ts );
    __mutex->unlock();
  } else {
    err = pthread_cond_timedwait( &(__cond_data->cond), &(__mutex->mutex_data->mutex), &ts );
  }

  if ( err == ETIMEDOUT ) {
    return false;
  } else if ( err != 0 ) {
    // some other error happened, a "real" error
    throw Exception(err, "Waiting for wait condition failed");
  } else {
    return true;
  }
}


/** Wait with relative timeout.
 * This waits for the given mutex until either a wakup signal is received or
 * the timeout has passed. The timeout has to be given in relative system time.
 * It is added to the current time and is then used similar to abstime_wait().
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
    struct timespec now;
#if defined(__MACH__) && defined(__APPLE__)
    struct timeval nowt;
    if ( gettimeofday(&nowt, NULL) != 0 ) {
      throw Exception(errno, "WaitCondition::reltimed_wait: Failed to get current time");
    }
    now.tv_sec  = nowt.tv_sec;
    now.tv_nsec = nowt.tv_usec * 1000;
#else
    if ( clock_gettime(CLOCK_REALTIME, &now) != 0 ) {
      throw Exception(errno, "WaitCondition::reltimed_wait: Failed to get current time");
    }
#endif

    long int s  = now.tv_sec  + sec;
    long int ns = now.tv_nsec + nanosec;
    if (ns >= 1000000000) {
      s  += 1;
      ns -= 1000000000;
    }

    struct timespec ts = { s, ns };
    long err = 0;

    if ( __own_mutex) {
      __mutex->lock();
      err = pthread_cond_timedwait( &(__cond_data->cond), &(__mutex->mutex_data->mutex), &ts );
      __mutex->unlock();
    } else {
      err = pthread_cond_timedwait( &(__cond_data->cond), &(__mutex->mutex_data->mutex), &ts );
    }

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
 * This wakes up any thread waiting for the condition, not a particular one.
 * No guarantee is given about the order of the woken up threads.
 * Note: If the internal mutex is used for this wait/wakeup cycle, the lock
 * to this mutex will be acquired during the wakeup, to ensure that all waiting
 * threads are woken up, even if a call to wait() and wake_one() overlapped.
 * If however, an external Mutex is used, you must ensure by yourself that it
 * is properly locked during the wakeup to ensure this.
 */
void
WaitCondition::wake_one()
{
  if (__own_mutex) {  // it's our internal mutex, lock!
    __mutex->lock();
    pthread_cond_signal( &(__cond_data->cond) );
    __mutex->unlock();
  } else {            // it's an external mutex, the user should care
    pthread_cond_signal( &(__cond_data->cond) );
  }
}


/** Wake up all waiting threads.
 * This wakes up all threads waiting for this condition.
 * Note: If the internal mutex is used for this wait/wakeup cycle, the lock
 * to this mutex will be acquired during the wakeup, to ensure that all waiting
 * threads are woken up, even if a call to wait() and wake_one() overlapped.
 * If however, an external Mutex is used, you must ensure by yourself that it
 * is properly locked during the wakeup to ensure this.
 */
void
WaitCondition::wake_all()
{
  if (__own_mutex) {  // it's our internal mutex, lock!
    __mutex->lock();
    pthread_cond_broadcast( &(__cond_data->cond) );
    __mutex->unlock();
  } else {            // it's an external mutex, the user should care
    pthread_cond_broadcast( &(__cond_data->cond) );
  }
}


} // end namespace fawkes
