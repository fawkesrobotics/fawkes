
/***************************************************************************
 *  mutex.h - Mutex
 *
 *  Generated: Thu Sep 14 16:58:49 2006
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

#ifndef _CORE_THREADING_MUTEX_H_
#define _CORE_THREADING_MUTEX_H_

#include "mutex_data.h"

#include <mutex>
#include <variant>

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

template <typename MutexT>
class MutexWrapper
{
public:
	MutexT &
	get_raw_mutex()
	{
		return mutex;
	}

	void
	lock()
	{
		mutex.lock();
#ifdef DEBUG_THREADING
		// do not switch order, lock holder must be protected with this mutex!
		mutex_data->set_lock_holder();
#endif
	}

	bool
	try_lock()
	{
		if (mutex.try_lock()) {
#ifdef DEBUG_THREADING
			mutex_data->set_lock_holder();
#endif
			return true;
		} else {
			return false;
		}
	}

	void
	unlock()
	{
#ifdef DEBUG_THREADING
		mutex_data->unset_lock_holder();
		// do not switch order, lock holder must be protected with this mutex!
#endif
		mutex.unlock();
	}

	void
	stopby()
	{
		lock();
		unlock();
	}

private:
	MutexT    mutex;
	MutexData mutex_data;
};

class Mutex : public MutexWrapper<std::mutex>
{
};

} // end namespace fawkes

#endif
