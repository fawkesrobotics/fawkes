
/***************************************************************************
 *  mutex_locker.h - Mutex locker
 *
 *  Created: Thu Oct 04 16:12:43 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef _CORE_THREADING_MUTEX_LOCKER_H_
#define _CORE_THREADING_MUTEX_LOCKER_H_

#include <core/threading/mutex.h>
#include <core/utils/refptr.h>

namespace fawkes {

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

template <class T_Mutex>
class MutexLocker
{
public:
	MutexLocker(RefPtr<T_Mutex> mutex, bool initially_lock = true);
	MutexLocker(T_Mutex *mutex, bool initially_lock = true);
	MutexLocker(T_Mutex &mutex, bool initially_lock = true);
	~MutexLocker();

	void relock();
	void unlock();

private:
	bool            locked_;
	RefPtr<T_Mutex> refmutex_;
	T_Mutex *       rawmutex_;
};

} // end namespace fawkes

#include "mutex_locker.hpp"

#endif
