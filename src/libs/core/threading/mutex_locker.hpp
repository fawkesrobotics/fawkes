
/***************************************************************************
 *  mutex_locker.cpp - mutex locker helper
 *
 *  Created: Thu Oct 04 16:14:30 2007
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

#ifndef _CORE_THREADING_MUTEX_LOCKER_HPP_
#define _CORE_THREADING_MUTEX_LOCKER_HPP_

#include <core/threading/mutex_locker.h>

namespace fawkes {

/** Constructor.
 * @param mutex Mutex to lock/unlock appropriately.
 * @param initially_lock true to lock the mutex in the constructor, false to not lock
 */
template <class T_Mutex>
MutexLocker<T_Mutex>::MutexLocker(RefPtr<T_Mutex> mutex, bool initially_lock)
{
	rawmutex_ = 0;
	refmutex_ = mutex;
	if (initially_lock) {
		refmutex_->lock();
	}
	locked_ = initially_lock;
}

/** Constructor.
 * @param mutex Mutex to lock/unlock appropriately.
 * @param initially_lock true to lock the mutex in the constructor, false to not lock
 */
template <class T_Mutex>
MutexLocker<T_Mutex>::MutexLocker(T_Mutex *mutex, bool initially_lock)
{
	rawmutex_ = mutex;
	if (initially_lock) {
		rawmutex_->lock();
	}
	locked_ = initially_lock;
}

/** Constructor.
 * @param mutex Mutex to lock/unlock appropriately.
 * @param initially_lock true to lock the mutex in the constructor, false to not lock
 */
template <class T_Mutex>
MutexLocker<T_Mutex>::MutexLocker(T_Mutex &mutex, bool initially_lock)
{
	rawmutex_ = &mutex;
	if (initially_lock) {
		rawmutex_->lock();
	}
	locked_ = initially_lock;
}

/** Destructor */
template <class T_Mutex>
MutexLocker<T_Mutex>::~MutexLocker()
{
	if (locked_) {
		if (rawmutex_) {
			rawmutex_->unlock();
		} else {
			refmutex_->unlock();
		}
	}
}

/** Lock this mutex, again.
 * Use this if you unlocked the mutex from the outside.
 */
template <class T_Mutex>
void
MutexLocker<T_Mutex>::relock()
{
	if (rawmutex_) {
		rawmutex_->lock();
	} else {
		refmutex_->lock();
	}
	locked_ = true;
}

/** Unlock the mutex. */
template <class T_Mutex>
void
MutexLocker<T_Mutex>::unlock()
{
	locked_ = false;
	if (rawmutex_) {
		rawmutex_->unlock();
	} else {
		refmutex_->unlock();
	}
}

} // end namespace fawkes

#endif /* ifndef _CORE_THREADING_MUTEX_LOCKER_HPP_ */
