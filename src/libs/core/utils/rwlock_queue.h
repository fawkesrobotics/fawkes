
/***************************************************************************
 *  rwlock_queue.h - Queue with read/write lock
 *
 *  Created: Tue Jan 13 16:36:52 2009
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

#ifndef __CORE_UTILS_RWLOCK_QUEUE_H_
#define __CORE_UTILS_RWLOCK_QUEUE_H_

#include <core/threading/read_write_lock.h>
#include <core/utils/refptr.h>
#include <queue>

namespace fawkes {


template <typename Type>
class RWLockQueue : public std::queue<Type>
{
 public:
  RWLockQueue();
  RWLockQueue(const RWLockQueue<Type> &ll);
  virtual ~RWLockQueue();

  void                   lock_for_read();
  void                   lock_for_write();
  bool                   try_lock_for_read();
  bool                   try_lock_for_write();
  void                   unlock();
  RefPtr<ReadWriteLock>  rwlock() const;

  void     push_locked(const Type& x);
  void     pop_locked();

  void clear();

  // not needed, no change to rwlock required (thus "incomplete" BigThree)
  //LockList<Type> &  operator=(const LockList<Type> &ll);
 private:
  RefPtr<ReadWriteLock> __rwlock;

};


/** @class RWLockQueue <core/utils/rwlock_queue.h>
 * Queue with a read/write lock.
 * This class provides a queue that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see ReadWriteLock
 * @ingroup FCL
 * @author Tim Niemueller
 */


/** Constructor. */
template <typename Type>
RWLockQueue<Type>::RWLockQueue()
{
  __rwlock = new ReadWriteLock();
}


/** Copy constructor.
 * @param ll RWLockQueue to copy
 */
template <typename Type>
RWLockQueue<Type>::RWLockQueue(const RWLockQueue<Type> &ll)
  : std::queue<Type>::queue(ll)
{
  __rwlock = new ReadWriteLock();
}


/** Destructor. */
template <typename Type>
RWLockQueue<Type>::~RWLockQueue()
{
  delete __rwlock;
}


/** Lock queue for reading. */
template <typename Type>
void
RWLockQueue<Type>::lock_for_read()
{
  __rwlock->lock_for_read();
}


/** Lock queue for writing. */
template <typename Type>
void
RWLockQueue<Type>::lock_for_write()
{
  __rwlock->lock_for_write();
}


/** Try to lock queue for reading.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename Type>
bool
RWLockQueue<Type>::try_lock_for_read()
{
  return __rwlock->try_lock_for_read();
}


/** Try to lock queue for writing.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename Type>
bool
RWLockQueue<Type>::try_lock_for_write()
{
  return __rwlock->try_lock_for_write();
}


/** Unlock list. */
template <typename Type>
void
RWLockQueue<Type>::unlock()
{
  return __rwlock->unlock();
}


/** Push element to queue with lock protection.
 * @param x element to add
 */
template <typename Type>
void
RWLockQueue<Type>::push_locked(const Type& x)
{
  __rwlock->lock_for_write();
  std::queue<Type>::push(x);
  __rwlock->unlock();
}


/** Pop element from queue with lock protection.
 */
template <typename Type>
void
RWLockQueue<Type>::pop_locked()
{
  __rwlock->lock_for_write();
  std::queue<Type>::pop();
  __rwlock->unlock();
}


/** Clear the queue. */
template <typename Type>
void
RWLockQueue<Type>::clear()
{
  __rwlock->lock_for_write();
  while ( ! std::queue<Type>::empty() ) {
    std::queue<Type>::pop();
  }
  __rwlock->unlock();
}


/** Get access to the internal rwlock.
 * Can be used with RwlockLocker.
 * @return internal rwlock
 */
template <typename Type>
ReadWriteLock *
RWLockQueue<Type>::rwlock() const
{
  return __rwlock;
}


} // end namespace fawkes

#endif
