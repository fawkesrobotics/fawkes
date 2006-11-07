
/***************************************************************************
 *  wait_condition.cpp - condition variable implementation
 *
 *  Generated: Thu Sep 14 21:43:30 2006
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

#include <core/threading/wait_condition.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_data.h>

#include <pthread.h>

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
 *     wait_condition->wakeOne();
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


/** Releases the given mutex and waits for the condition. The mutex must
 * be locked before calling this method. Otherwise it will return immediately
 * with value false.
 * @param mutex the mutex to unlock
 * @param timeout_sec Optional timeout in seconds that we will wait at most before
 * we go on. If timeout_sec and timeout_nanosec equal zero waits forever.
 * @param timeout_nanosec Optional timeout in nanoseconds that we will wait at most before
 * we go on. If timeout_sec and timeout_nanosec equal zero waits forever.
 * @return true, if the thread was woken up by another thread calling wakeOne() or
 * wakeAll(). False, if the mutex was unlocked or the timeout has been reached waiting
 * for the condition.
 */
bool
WaitCondition::wait(Mutex *mutex, unsigned int timeout_sec, unsigned int timeout_nanosec)
{
  if ( (timeout_sec > 0) || (timeout_nanosec > 0) ) {
    struct timespec ts = { timeout_sec, timeout_nanosec };
    int err;
    err = pthread_cond_timedwait( &(cond_data->cond), &(mutex->mutex_data->mutex), &ts );
    if ( err == 0 ) {
      return true;
    } else {
      return false;
    }
  } else {
    pthread_cond_wait( &(cond_data->cond), &(mutex->mutex_data->mutex) );
    return true;
  }
}


/** Wake another thread waiting for this condition.
 * This wakes up any thread waiting for the condition, not a particular one. No guarantee
 * is given about the order of the woken up threads.
 */
void
WaitCondition::wakeOne()
{
  pthread_mutex_lock( &(cond_data->mutex) );
  pthread_cond_signal( &(cond_data->cond) );
  pthread_mutex_unlock( &(cond_data->mutex) );
}


/** Wake up all waiting threads.
 * This wakes up all threads waiting for this condition.
 */
void
WaitCondition::wakeAll()
{
  pthread_mutex_lock( &(cond_data->mutex) );
  pthread_cond_broadcast( &(cond_data->cond) );
  pthread_mutex_unlock( &(cond_data->mutex) );
}

