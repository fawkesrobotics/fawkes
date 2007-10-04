
/***************************************************************************
 *  mutex_locker.cpp - mutex locker helper
 *
 *  Created: Thu Oct 04 16:14:30 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/mutex_locker.h>
#include <core/threading/mutex.h>

/** @class MutexLocker <core/threading/mutex_locker.h>
 * Mutex locking helper.
 * This class is a convenience function which can help you prevent a quite
 * a few headaches. Consider the following code.
 * @code
 * void my_function()
 * {
 *   mutex->lock();
 *   for (int i = 0; i < LIMIT; ++i) {
 *     if ( failure ) {
 *       mutex->unlock
 *     }
 *   }
 *
 *   switch ( someval ) {
 *     VALA:
 *       mutex->unlock();
 *       return;
 *     VALB:
 *       do_something();
 *   }
 *
 *   try {
 *     do_function_that_throws_exceptions();
 *   } catch (Exception &e) {
 *     mutex->unlock();
 *     throw;
 *   }
 *   mutex->unlock();
 * }
 * @endcode
 * This is not a complete list of examples but as you see if you have many
 * exit points in a function it becomes more and more work to have correct
 * locking behavior.
 *
 * This is a lot simpler with the MutexLocker. The MutexLocker locks the
 * given mutex on creation, and unlocks it in the destructor. If you now
 * have a mutex locker on the stack as integral type the destructor is
 * called automagically on function exit and thus the mutex is appropriately
 * unlocked.
 * The code would look like this:
 * @code
 * void my_function()
 * {
 *   MutexLocker ml(mutex);
 *   // do anything, no need to call mutex->lock()/unlock() if only has to be
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
 * @param mutex Mutex to lock/unlock appropriately.
 */
MutexLocker::MutexLocker(Mutex *mutex)
{
  this->mutex = mutex;
  mutex->lock();
}

/** Destructor */
MutexLocker::~MutexLocker()
{
  mutex->tryLock();
  mutex->unlock();
}


/** Lock this mutex, again.
 * Use this if you unlocked the mutex from the outside.
 */
void
MutexLocker::relock()
{
  mutex->lock();
}


/** Unlock the mutex. */
void
MutexLocker::unlock()
{
  mutex->unlock();
}
