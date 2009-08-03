
/***************************************************************************
 *  spinlock.h - Spinlock
 *
 *  Created: Wed Apr 02 13:20:31 2008
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

#include <core/threading/spinlock.h>
#include <core/threading/thread.h>
#include <core/exception.h>

#include <pthread.h>
#include <unistd.h>

// cf. http://people.redhat.com/drepper/posix-option-groups.html
#if defined(_POSIX_SPIN_LOCKS) && (_POSIX_SPIN_LOCKS - 200112L) >= 0
#  define USE_POSIX_SPIN_LOCKS
#else
#  undef USE_POSIX_SPIN_LOCKS
#  include <core/threading/mutex.h>
#endif

namespace fawkes {

/** @class Spinlock <core/threading/spinlock.h>
 * Spin lock.
 * This class is similar to a Mutex in that it is used in a multi-threading
 * environment to lock access to resources.
 *
 * The difference is that the spinlock will do a busy waiting until it acquires
 * the lock while the mutex would block and wait, and may even starve if another
 * threads releases a lock only for a short period of time.
 *
 * Spinlocks are risky, priority inversion may be caused if used improperly.
 * Be sure what you are doing if you use spinlocks.
 *
 * @ingroup Threading
 * @ingroup FCL
 *
 * @author Tim Niemueller
 */

/// @cond INTERNALS
class SpinlockData
{
 public:
#ifdef USE_POSIX_SPIN_LOCKS
  pthread_spinlock_t spinlock;
#else
  Mutex              mutex;
#endif
};
/// @endcond


/** Constructor */
Spinlock::Spinlock()
{
  spinlock_data = new SpinlockData();
#ifdef USE_POSIX_SPIN_LOCKS
  pthread_spin_init(&(spinlock_data->spinlock), PTHREAD_PROCESS_PRIVATE);
#endif
}

/** Destructor */
Spinlock::~Spinlock()
{
#ifdef USE_POSIX_SPIN_LOCKS
  pthread_spin_destroy(&(spinlock_data->spinlock));
#endif
  delete spinlock_data;
  spinlock_data = NULL;
}


/** Lock this spinlock.
 * A call to lock() will block until the lock on the spinlock could be aquired.
 * If you want to avoid see consider using try_lock().
 */
void
Spinlock::lock()
{
#ifdef USE_POSIX_SPIN_LOCKS
  int err = 0;
  if ( (err = pthread_spin_lock(&(spinlock_data->spinlock))) != 0 ) {
    throw Exception(err, "Failed to aquire lock for thread %s", Thread::current_thread()->name());
  }
#else
  bool locked = false;
  while ( ! locked ) {
    locked = spinlock_data->mutex.try_lock();
  }
#endif
}


/** Tries to lock the spinlock.
 * This can also be used to check if a spinlock is locked. The code for this
 * can be:
 *
 * @code
 * bool locked = false;
 * if ( spinlock->try_lock() ) {
 *   spinlock->unlock();
 *   locked = true;
 * }
 * @endcode
 *
 * This cannot be implemented in Spinlock in a locked() method since this
 * would lead to race conditions in many situations.
 *
 * @return true, if the spinlock could be locked, false otherwise.
 */
bool
Spinlock::try_lock()
{
#ifdef USE_POSIX_SPIN_LOCKS
  if (pthread_spin_trylock(&(spinlock_data->spinlock)) == 0) {
    return true;
  } else {
    return false;
  }
#else
  return spinlock_data->mutex.try_lock();
#endif
}


/** Unlock the spinlock. */
void
Spinlock::unlock()
{
#ifdef USE_POSIX_SPIN_LOCKS
  pthread_spin_unlock(&(spinlock_data->spinlock));
#else
  spinlock_data->mutex.unlock();
#endif
}


} // end namespace fawkes
