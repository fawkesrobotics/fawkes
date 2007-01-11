
/***************************************************************************
 *  lock_list.h - Lockable list
 *
 *  Created: Tue Oct 31 18:25:03 2006
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

#ifndef __CORE_UTILS_LOCK_LIST_H_
#define __CORE_UTILS_LOCK_LIST_H_

#include <core/threading/mutex.h>
#include <list>

template <typename Type>
class LockList : public std::list<Type>
{
 public:
  LockList();
  LockList(const LockList<Type> &ll);
  virtual ~LockList();

  void lock();
  bool tryLock();
  void unlock();

  void push_back_locked(const Type& x);
  void push_front_locked(const Type& x);

  // not needed, no change to mutex required (thus "incomplete" BigThree)
  //LockList<Type> &  operator=(const LockList<Type> &ll);
 private:
  Mutex *mutex;

};


/** @class LockList core/utils/lock_list.h
 * List with a lock.
 * This class provides a list that has an intrinsic lock. The lock can be applied
 * with the regular locking methods.
 *
 * @see Mutex
 * @author Tim Niemueller
 */


/** Constructor. */
template <typename Type>
LockList<Type>::LockList()
{
  mutex = new Mutex();
}


/** Copy constructor.
 * @param ll LockList to copy
 */
template <typename Type>
LockList<Type>::LockList(const LockList<Type> &ll)
  : std::list<Type>::list(ll)
{
  mutex = new Mutex();
}


/** Destructor. */
template <typename Type>
LockList<Type>::~LockList()
{
  delete mutex;
}


/** Lock list. */
template <typename Type>
void
LockList<Type>::lock()
{
  mutex->lock();
}


/** Try to lock list.
 * @return true, if the lock has been aquired, false otherwise.
 */
template <typename Type>
bool
LockList<Type>::tryLock()
{
  return mutex->tryLock();
}


/** Unlock list. */
template <typename Type>
void
LockList<Type>::unlock()
{
  return mutex->unlock();
}


/** Push element to list at back with lock protection.
 * @param x element to add
 */
template <typename Type>
void
LockList<Type>::push_back_locked(const Type& x)
{
  mutex->lock();
  std::list<Type>::push_back(x);
  mutex->unlock();
}


/** Push element to list at front with lock protection.
 * @param x element to add
 */
template <typename Type>
void
LockList<Type>::push_front_locked(const Type& x)
{
  mutex->lock();
  std::list<Type>::push_front(x);
  mutex->unlock();
}

#endif
