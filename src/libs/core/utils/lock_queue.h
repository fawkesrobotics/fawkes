
/***************************************************************************
 *  lock_queue.h - Lockable queue
 *
 *  Created: Mon Nov 20 15:40:40 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __CORE_UTILS_LOCK_QUEUE_H_
#define __CORE_UTILS_LOCK_QUEUE_H_

#include <core/threading/mutex.h>
#include <queue>

template <typename Type>
class LockQueue : public std::queue<Type>
{
 public:
  LockQueue();
  LockQueue(const LockQueue<Type> &ll);
  virtual ~LockQueue();

  void lock();
  bool tryLock();
  void unlock();

  void push_locked(const Type& x);
  void pop_locked();

  void clear();

  // not needed, no change to mutex required (thus "incomplete" BigThree)
  //LockList<Type> &  operator=(const LockList<Type> &ll);
 private:
  Mutex *mutex;

};


/** @class LockQueue core/utils/lock_queue.h
 * Queue with a lock.
 * This class provides a queue that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see Mutex
 * @ingroup FCL
 * @author Tim Niemueller
 */


/** Constructor. */
template <typename Type>
LockQueue<Type>::LockQueue()
{
  mutex = new Mutex();
}


/** Copy constructor.
 * @param ll LockQueue to copy
 */
template <typename Type>
LockQueue<Type>::LockQueue(const LockQueue<Type> &ll)
  : std::queue<Type>::queue(ll)
{
  mutex = new Mutex();
}


/** Destructor. */
template <typename Type>
LockQueue<Type>::~LockQueue()
{
  delete mutex;
}


/** Lock queue. */
template <typename Type>
void
LockQueue<Type>::lock()
{
  mutex->lock();
}


/** Try to lock queue.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename Type>
bool
LockQueue<Type>::tryLock()
{
  return mutex->tryLock();
}


/** Unlock list. */
template <typename Type>
void
LockQueue<Type>::unlock()
{
  return mutex->unlock();
}


/** Push element to queue with lock protection.
 * @param x element to add
 */
template <typename Type>
void
LockQueue<Type>::push_locked(const Type& x)
{
  mutex->lock();
  std::queue<Type>::push(x);
  mutex->unlock();
}


/** Pop element from queue with lock protection.
 */
template <typename Type>
void
LockQueue<Type>::pop_locked()
{
  mutex->lock();
  std::queue<Type>::pop();
  mutex->unlock();
}

/** Clear the queue. */
template <typename Type>
void
LockQueue<Type>::clear()
{
  mutex->lock();
  while ( ! std::queue<Type>::empty() ) {
    std::queue<Type>::pop();
  }
  mutex->unlock();
}

#endif
