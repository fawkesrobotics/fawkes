
/***************************************************************************
 *  mutex.cpp - implementation of mutex, based on pthreads
 *
 *  Generated: Thu Sep 14 17:03:57 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/mutex.h>
#include <core/threading/mutex_data.h>
#include <core/threading/thread.h>
#include <core/exception.h>

#include <pthread.h>

namespace fawkes {

/** @class Mutex core/threading/mutex.h
 * Mutex mutual exclusion lock.
 * This class is used in a multi-threading environment to lock access to
 * resources. This is needed to prevent two threads from modifying a value
 * at the same time or to prevent a thread from getting a dirty copy of
 * a piece of data (the reader reads while a writer is writing, this could
 * leave the data in a state where the reader reads half of the new and half
 * of the old data).
 *
 * As a rule of thumb you should lock the mutex as short as possible and as
 * long as needed. Locking the mutex too long will lead in a bad performance
 * of the multi-threaded application because many threads are waiting for
 * the lock and are not doing anything useful.
 * If you do not lock enough code (and so serialize it) it will cause pain
 * and errors.
 *
 * @ingroup Threading
 * @ingroup FCL
 * @see example_mutex_count.cpp
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * @param type mutex type 
 */
Mutex::Mutex(Type type)
{
  mutex_data = new MutexData();

  pthread_mutexattr_t attr;
  pthread_mutexattr_init(&attr);
  if (type == RECURSIVE) {
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
  } else {
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_NORMAL);
  }

  pthread_mutex_init(&(mutex_data->mutex), &attr);
}

/** Destructor */
Mutex::~Mutex()
{
  pthread_mutex_destroy(&(mutex_data->mutex));
  delete mutex_data;
  mutex_data = NULL;
}


/** Lock this mutex.
 * A call to lock() will block until the lock on the mutex could be aquired.
 * If you want to avoid see consider using try_lock().
 */
void
Mutex::lock()
{
  int err = 0;
  if ( (err = pthread_mutex_lock(&(mutex_data->mutex))) != 0 ) {
    throw Exception(err, "Failed to aquire lock for thread %s", Thread::current_thread()->name());
  }
#ifdef DEBUG_THREADING
  // do not switch order, lock holder must be protected with this mutex!
  mutex_data->set_lock_holder();
#endif
}


/** Tries to lock the mutex.
 * This can also be used to check if a mutex is locked. The code for this
 * can be:
 *
 * @code
 * bool locked = false;
 * if ( mutex->try_lock() ) {
 *   mutex->unlock();
 *   locked = true;
 * }
 * @endcode
 *
 * This cannot be implemented in Mutex in a locked() method since this
 * would lead to race conditions in many situations.
 *
 * @return true, if the mutex could be locked, false otherwise.
 */
bool
Mutex::try_lock()
{
  if (pthread_mutex_trylock(&(mutex_data->mutex)) == 0) {
#ifdef DEBUG_THREADING
    mutex_data->set_lock_holder();
#endif
    return true;
  } else {
    return false;
  }
}


/** Unlock the mutex. */
void
Mutex::unlock()
{
#ifdef DEBUG_THREADING
  mutex_data->unset_lock_holder();
  // do not switch order, lock holder must be protected with this mutex!
#endif
  pthread_mutex_unlock(&(mutex_data->mutex));
}


/** Shortly stop by at the mutex.
 * This will just lock and unlock the mutex. It is equivalent to
 * @code
 *   mutex->lock();
 *   mutex->unlock();
 * @endcode
 * This can be handy if you have to protect starvation and just have a stop-by
 * mutex.
 */
void
Mutex::stopby()
{
  pthread_mutex_lock(&(mutex_data->mutex));
  pthread_mutex_unlock(&(mutex_data->mutex));
}


} // end namespace fawkes
