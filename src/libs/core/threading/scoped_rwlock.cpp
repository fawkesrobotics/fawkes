
/***************************************************************************
 *  scoped_rwlock.cpp - Scoped read/write lock
 *
 *  Created: Mon Jan 10 11:45:49 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/scoped_rwlock.h>
#include <core/threading/read_write_lock.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ScopedRWLock <core/threading/scoped_rwlock.h>
 * Scoped read/write lock.
 * This class is a convenience class which can help you prevent quite
 * a few headaches. Consider the following code.
 * @code
 * void my_function()
 * {
 *   rwlock->lock_for_write();
 *   for (int i = 0; i < LIMIT; ++i) {
 *     if ( failure ) {
 *       rwlock->unlock();
 *     }
 *   }
 *
 *   switch ( someval ) {
 *     VALA:
 *       rwlock->unlock();
 *       return;
 *     VALB:
 *       do_something();
 *   }
 *
 *   try {
 *     do_function_that_throws_exceptions();
 *   } catch (Exception &e) {
 *     rwlock->unlock();
 *     throw;
 *   }
 *   rwlock->unlock();
 * }
 * @endcode
 * This is not a complete list of examples but as you see if you have many
 * exit points in a function it becomes more and more work to have correct
 * locking behavior.
 *
 * This is a lot simpler with the ScopedRWLock. The ScopedRWLock locks the
 * given ReadWriteLock on creation, and unlocks it in the destructor. If you now
 * have a read/write locker on the stack as integral type the destructor is
 * called automagically on function exit and thus the lock is appropriately
 * unlocked.
 * The code would look like this:
 * @code
 * void my_function()
 * {
 *   ScopedRWLock lock(rwlock);
 *   // do anything, no need to call rwlock->lock_*()/unlock() if only has to be
 *   // called on entering and exiting the function.
 * }
 * @endcode
 *
 * @ingroup Threading
 * @ingroup FCL
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * @param rwlock ReadWriteLock to lock/unlock appropriately.
 * @param initially_lock true to lock the rwlock in the constructor,
 * false to not lock
 * @param lock_type locking type, lock either for writing or for reading
 */
ScopedRWLock::ScopedRWLock(RefPtr<ReadWriteLock> rwlock, ScopedRWLock::LockType lock_type,
                           bool initially_lock)
{
  __rawrwlock = 0;
  __refrwlock = rwlock;
  __lock_type = lock_type;
  if ( initially_lock ) {
    if (__lock_type == LOCK_WRITE) {
      __refrwlock->lock_for_write();
    } else {
      __refrwlock->lock_for_read();
    }
  }
  __locked = initially_lock;
}


/** Constructor.
 * @param rwlock ReadWriteLock to lock/unlock appropriately.
 * @param initially_lock true to lock the rwlock in the constructor,
 * false to not lock
 * @param lock_type locking type, lock either for writing or for reading
 */
ScopedRWLock::ScopedRWLock(ReadWriteLock *rwlock, ScopedRWLock::LockType lock_type,
                           bool initially_lock)
{
  __rawrwlock = rwlock;
  __lock_type = lock_type;
  if ( initially_lock ) {
    if (__lock_type == LOCK_WRITE) {
      __rawrwlock->lock_for_write();
    } else {
      __rawrwlock->lock_for_read();
    }
  }
  __locked = initially_lock;
}


/** Destructor */
ScopedRWLock::~ScopedRWLock()
{
  if ( __locked ) {
    if ( __rawrwlock) {
      __rawrwlock->unlock();
    } else {
      __refrwlock->unlock();
    }
  }
}


/** Lock this rwlock, again.
 * Use this if you unlocked the rwlock from the outside.
 */
void
ScopedRWLock::relock()
{
  if ( __rawrwlock ) {
    if (__lock_type == LOCK_WRITE) {
      __rawrwlock->lock_for_write();
    } else {
      __rawrwlock->lock_for_read();
    }
  } else {
    if (__lock_type == LOCK_WRITE) {
      __refrwlock->lock_for_write();
    } else {
      __refrwlock->lock_for_read();
    }
  }
  __locked = true;
}


/** Unlock the rwlock. */
void
ScopedRWLock::unlock()
{
  __locked = false;
  if ( __rawrwlock ) {
    __rawrwlock->unlock();
  } else {
    __refrwlock->unlock();
  }
}


} // end namespace fawkes
