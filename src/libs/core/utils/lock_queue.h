
/***************************************************************************
 *  lock_queue.h - Lockable queue
 *
 *  Created: Mon Nov 20 15:40:40 2006
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

#ifndef __CORE_UTILS_LOCK_QUEUE_H_
#define __CORE_UTILS_LOCK_QUEUE_H_

#include <core/threading/mutex.h>
#include <core/utils/refptr.h>
#include <queue>

namespace fawkes {

/** @class LockQueue <core/utils/lock_queue.h>
 * Queue with a lock.
 * This class provides a queue that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see Mutex
 * @ingroup FCL
 * @author Tim Niemueller
 */
template <typename Type>
class LockQueue : public std::queue<Type>
{
 public:
  /** Constructor. */
  LockQueue();

  /** Copy constructor.
   * @param ll LockQueue to copy
   */
  LockQueue(const LockQueue<Type> &ll);

  /** Destructor. */
  virtual ~LockQueue();

  /** Lock queue. */
  void           lock() const;

  /** Try to lock queue.
   * @return true, if the lock has been aquired, false otherwise.
   */
  bool           try_lock() const;

  /** Unlock list. */
  void           unlock() const;

  /** Get access to the internal mutex.
   * Can be used with MutexLocker.
   * @return internal mutex
   */
  RefPtr<Mutex>  mutex() const
  { return __mutex; }

  /** Push element to queue with lock protection.
   * @param x element to add
   */
  void     push_locked(const Type& x);

  /** Pop element from queue with lock protection. */
  void     pop_locked();

  /** Clear the queue. */
  void clear();

  // not needed, no change to mutex required (thus "incomplete" BigThree)
  //LockList<Type> &  operator=(const LockList<Type> &ll);
 private:
  mutable RefPtr<Mutex> __mutex;

};




template <typename Type>
LockQueue<Type>::LockQueue()
  : __mutex(new Mutex())
{}


template <typename Type>
LockQueue<Type>::LockQueue(const LockQueue<Type> &ll)
  : std::queue<Type>::queue(ll), __mutex(new Mutex())
{}


template <typename Type>
LockQueue<Type>::~LockQueue()
{}


template <typename Type>
void
LockQueue<Type>::lock() const
{
  __mutex->lock();
}


template <typename Type>
bool
LockQueue<Type>::try_lock() const
{
  return __mutex->try_lock();
}


template <typename Type>
void
LockQueue<Type>::unlock() const
{
  return __mutex->unlock();
}


template <typename Type>
void
LockQueue<Type>::push_locked(const Type& x)
{
  __mutex->lock();
  std::queue<Type>::push(x);
  __mutex->unlock();
}


template <typename Type>
void
LockQueue<Type>::pop_locked()
{
  __mutex->lock();
  std::queue<Type>::pop();
  __mutex->unlock();
}

template <typename Type>
void
LockQueue<Type>::clear()
{
  __mutex->lock();
  while ( ! std::queue<Type>::empty() ) {
    std::queue<Type>::pop();
  }
  __mutex->unlock();
}


} // end namespace fawkes

#endif
